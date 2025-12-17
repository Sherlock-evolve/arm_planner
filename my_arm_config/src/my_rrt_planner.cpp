#include <ompl/base/GoalSampleableRegion.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
namespace geometric
{

// A simple RRT implementation that can be customized inside solve().
class MyRRT : public base::Planner
{
public:
  explicit MyRRT(const base::SpaceInformationPtr & si)
  : base::Planner(si, "MyRRT")
  {
    specs_.approximateSolutions = true;
    specs_.directed = false;
  }

  ~MyRRT() override
  {
    freeMemory();
  }

  void setRange(double distance)
  {
    maxDistance_ = distance;
  }

  double getRange() const
  {
    return maxDistance_;
  }

  void setGoalBias(double bias)
  {
    goalBias_ = bias;
  }

  double getGoalBias() const
  {
    return goalBias_;
  }

  void clear() override
  {
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_) {
      nn_->clear();
    }
  }

  void setup() override
  {
    Planner::setup();
    if (!nn_) {
      nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    }
    nn_->setDistanceFunction([this](const Motion * a, const Motion * b) {
      return si_->distance(a->state, b->state);
    });

    tools::SelfConfig sc(si_, getName());
    if (maxDistance_ <= 0.0) {
      sc.configurePlannerRange(maxDistance_);
    }
  }

  base::PlannerStatus solve(const base::PlannerTerminationCondition & ptc) override
  {
    checkValidity();

    auto * goal = pdef_->getGoal().get();
    auto * goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (!goal_s) {
      OMPL_ERROR("%s: Unknown goal type", getName().c_str());
      return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State * st = pis_.nextStart()) {
      auto * motion = new Motion(si_);
      si_->copyState(motion->state, st);
      nn_->add(motion);
    }

    if (nn_->size() == 0) {
      OMPL_ERROR("%s: There are no valid start states", getName().c_str());
      return base::PlannerStatus::INVALID_START;
    }

    auto * rmotion = new Motion(si_);
    auto * xstate = si_->allocState();

    sampler_ = si_->allocStateSampler();

    base::PlannerStatus solved = base::PlannerStatus::TIMEOUT;
    Motion * solution = nullptr;
    Motion * approxSol = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    while (!ptc)
    {
      // Sample: goal-biased
      if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
        goal_s->sampleGoal(rmotion->state);
      } else {
        sampler_->sampleUniform(rmotion->state);
      }

      // Nearest neighbor
      Motion * nmotion = nn_->nearest(rmotion);

      // Steer toward sample with step limited by maxDistance_
      double d = si_->distance(nmotion->state, rmotion->state);
      if (d > maxDistance_) {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, xstate);
      } else {
        si_->copyState(xstate, rmotion->state);
      }

      // Collision check
      if (si_->checkMotion(nmotion->state, xstate)) {
        auto * newmotion = new Motion(si_);
        si_->copyState(newmotion->state, xstate);
        newmotion->parent = nmotion;
        nn_->add(newmotion);

        double dist = 0.0;
        bool sat = goal->isSatisfied(newmotion->state, &dist);
        if (sat) {
          solution = newmotion;
          approxDist = dist;
          solved = base::PlannerStatus::EXACT_SOLUTION;
          break;
        } else if (dist < approxDist) {
          approxSol = newmotion;
          approxDist = dist;
        }
      }
    }

    bool addedSolution = false;
    if (solution || approxSol) {
      Motion * m = solution ? solution : approxSol;
      // Reconstruct path
      std::vector<Motion *> mpath;
      while (m != nullptr) {
        mpath.push_back(m);
        m = m->parent;
      }
      auto path(std::make_shared<PathGeometric>(si_));
      for (auto it = mpath.rbegin(); it != mpath.rend(); ++it) {
        path->append((*it)->state);
      }
      pdef_->addSolutionPath(path, solution != nullptr, approxDist, getName());
      addedSolution = true;
    }

    si_->freeState(xstate);
    delete rmotion;

    return solved ? solved : (addedSolution ? base::PlannerStatus::APPROXIMATE_SOLUTION
                                            : base::PlannerStatus::TIMEOUT);
  }

protected:
  class Motion
  {
  public:
    Motion() = default;
    explicit Motion(const base::SpaceInformationPtr & si) : state(si->allocState()) {}
    ~Motion() = default;
    base::State * state{nullptr};
    Motion * parent{nullptr};
  };

  void freeMemory()
  {
    if (!nn_) return;
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto * m : motions) {
      if (m->state) {
        si_->freeState(m->state);
      }
      delete m;
    }
  }

  base::StateSamplerPtr sampler_;
  std::shared_ptr<NearestNeighbors<Motion *>> nn_;
  double goalBias_{0.05};
  double maxDistance_{0.0};
  RNG rng_;
};

}  // namespace geometric
}  // namespace ompl

