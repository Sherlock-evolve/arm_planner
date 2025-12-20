#include <moveit/planners/ompl/ompl_planner_manager.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pluginlib/class_list_macros.hpp>

// Include implementation of custom planner
#include "my_rrt_planner.cpp"

namespace my_arm_config
{

class MyOMPLPlannerManager : public ompl_interface::OMPLPlannerManager
{
public:
  MyOMPLPlannerManager() = default;
  ~MyOMPLPlannerManager() override = default;

  bool initialize(const robot_model::RobotModelConstPtr & model, const std::string & ns) override
  {
    if (!ompl_interface::OMPLPlannerManager::initialize(model, ns)) {
      return false;
    }

    // Register the custom planner so it can be referenced by type name in ompl_planning.yaml
    ompl_interface::PlannerAllocator allocator =
      [](const ompl::base::SpaceInformationPtr & si, const std::string &) {
        return std::make_shared<ompl::geometric::MyRRT>(si);
      };
    planner_interfaces_.registerPlannerAllocator("geometric::MyRRT", allocator);
    return true;
  }
};

}  // namespace my_arm_config

PLUGINLIB_EXPORT_CLASS(my_arm_config::MyOMPLPlannerManager, planning_interface::PlannerManager)


