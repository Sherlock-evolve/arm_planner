#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "my_arm_config/action/plan_execute_pose.hpp"

namespace my_arm_config
{

using PlanExecutePose = my_arm_config::action::PlanExecutePose;
using GoalHandlePlanExecutePose = rclcpp_action::ServerGoalHandle<PlanExecutePose>;

class MoveItMotionServer : public rclcpp::Node
{
public:
  explicit MoveItMotionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("moveit_motion_server", options)
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "manipulator");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "world");
    action_server_ = rclcpp_action::create_server<PlanExecutePose>(
      this,
      "plan_execute_pose",
      std::bind(&MoveItMotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveItMotionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveItMotionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveIt motion server ready for group '%s'", planning_group_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlanExecutePose::Goal> goal)
  {
    if (!goal) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto & pose = goal->target_pose.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z)) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: pose contains NaN/Inf");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    (void)goal_handle;
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (move_group_) {
      move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    std::thread(&MoveItMotionServer::execute, this, goal_handle).detach();
  }

  void ensure_move_group()
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!move_group_) {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      move_group_->setPoseReferenceFrame(base_frame_);
    }
  }

  static std::string error_code_to_string(const moveit::core::MoveItErrorCode & code)
  {
    using moveit_msgs::msg::MoveItErrorCodes;
    switch (code.val) {
      case MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
      case MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
      case MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
      case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "INVALIDATED_BY_ENVIRONMENT_CHANGE";
      case MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
      case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_ACQUIRE_SENSOR_DATA";
      case MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
      case MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";
      case MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
      case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_CONSTRAINTS";
      case MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
      case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_CONSTRAINTS";
      case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";
      case MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
      case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
      case MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
      case MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";
      default:
        return "MoveIt error code " + std::to_string(code.val);
    }
  }

  void execute(const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    auto result = std::make_shared<PlanExecutePose::Result>();
    auto goal = goal_handle->get_goal();

    ensure_move_group();

    geometry_msgs::msg::PoseStamped target_pose = goal->target_pose;
    if (target_pose.header.frame_id.empty()) {
      target_pose.header.frame_id = base_frame_;
    }
    target_pose.header.stamp = this->now();

    auto feedback = std::make_shared<PlanExecutePose::Feedback>();
    feedback->status = "Planning started";
    feedback->progress = 0.1f;
    goal_handle->publish_feedback(feedback);

    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
      move_group_->setPoseTarget(target_pose.pose);
      const double vel = std::clamp(goal->velocity_scaling, 0.01, 1.0);
      const double acc = std::clamp(goal->acceleration_scaling, 0.01, 1.0);
      move_group_->setMaxVelocityScalingFactor(vel);
      move_group_->setMaxAccelerationScalingFactor(acc);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      planning_result = move_group_->plan(plan);
    }

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      std::string error_msg = error_code_to_string(planning_result);
      result->message = "Planning failed: " + error_msg;
      
      if (planning_result.val == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED ||
          planning_result.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN) {
        result->message += "\n可能原因：目标位姿不可达、IK 求解失败、碰撞检测失败或路径规划失败";
        result->message += "\n提示：即使输入值在范围内，规划仍可能因上述原因失败";
      } else if (planning_result.val == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION) {
        result->message += "\n原因：无法找到逆运动学解，目标位姿超出工作空间";
      } else if (planning_result.val == moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION) {
        result->message += "\n原因：目标位姿与环境或自身发生碰撞";
      } else if (planning_result.val == moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION) {
        result->message += "\n原因：起始状态发生碰撞，请检查当前机械臂状态";
      }
      
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled after planning";
      goal_handle->canceled(result);
      return;
    }

    feedback->status = "Planning succeeded";
    feedback->progress = 0.6f;
    goal_handle->publish_feedback(feedback);

    if (!goal->execute) {
      result->success = true;
      result->message = "Plan available (execution skipped)";
      goal_handle->succeed(result);
      return;
    }

    feedback->status = "Executing trajectory";
    feedback->progress = 0.8f;
    goal_handle->publish_feedback(feedback);

    moveit::core::MoveItErrorCode exec_result;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      const bool wait = goal->wait_for_execution;
      if (wait) {
        exec_result = move_group_->execute(plan);
      } else {
        exec_result = move_group_->asyncExecute(plan);
      }
    }

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      result->message = "Execution failed: " + error_code_to_string(exec_result);
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled during execution";
      goal_handle->canceled(result);
      return;
    }

    feedback->status = "Execution completed";
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Motion completed successfully";
    goal_handle->succeed(result);
  }

  std::string planning_group_;
  std::string base_frame_;
  rclcpp_action::Server<PlanExecutePose>::SharedPtr action_server_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex move_group_mutex_;
};

}  // namespace my_arm_config

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_arm_config::MoveItMotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

