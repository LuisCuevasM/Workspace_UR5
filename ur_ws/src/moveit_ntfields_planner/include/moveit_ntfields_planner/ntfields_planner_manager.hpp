#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_ntfields_planner
{

class NTFieldsPlannerManager : public planning_interface::PlannerManager
{
public:
  NTFieldsPlannerManager() = default;
  ~NTFieldsPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

private:
  bool supportsOnlyJointGoals(const planning_interface::MotionPlanRequest& req) const;

  moveit::core::RobotModelConstPtr model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::string group_name_ = "ur_arm";
  std::string planner_id_ = "NTFieldsJointInterpolation";
  std::string planner_url_ = "http://172.19.0.2:8888/plan";
  double request_timeout_ = 60.0;
  double default_dt_ = 0.15;
  std::size_t interpolation_steps_ = 10;
};

}  // namespace moveit_ntfields_planner
