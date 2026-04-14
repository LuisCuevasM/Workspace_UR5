#pragma once

#include <atomic>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_ntfields_planner
{

class NTFieldsPlanningContext : public planning_interface::PlanningContext
{
public:
  NTFieldsPlanningContext(const std::string& name, const std::string& group,
                          const moveit::core::RobotModelConstPtr& model,
                          const moveit::core::JointModelGroup* joint_model_group,
                          const rclcpp::Node::SharedPtr& node, std::string planner_url, double request_timeout,
                          double default_dt, std::size_t interpolation_steps);

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear() override;

private:
  bool extractStart(const moveit::core::RobotState& seed_state, std::vector<double>& q_start) const;
  bool extractGoal(std::vector<double>& q_goal) const;
  bool requestRemotePlan(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                         std::vector<std::vector<double>>& waypoints) const;
  bool buildInterpolatedPath(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                             std::vector<std::vector<double>>& waypoints) const;
  bool buildTrajectory(const std::vector<std::vector<double>>& waypoints,
                       robot_trajectory::RobotTrajectoryPtr& trajectory) const;

  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  rclcpp::Node::SharedPtr node_;
  std::string planner_url_;
  double request_timeout_;
  double default_dt_;
  std::size_t interpolation_steps_;
  std::atomic_bool terminate_requested_{ false };
};

}  // namespace moveit_ntfields_planner
