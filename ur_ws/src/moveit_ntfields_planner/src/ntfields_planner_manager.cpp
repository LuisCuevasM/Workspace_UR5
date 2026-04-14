#include <moveit_ntfields_planner/ntfields_planner_manager.hpp>

#include <moveit_ntfields_planner/ntfields_planning_context.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit_ntfields_planner
{

namespace
{
bool constraintsEmpty(const moveit_msgs::msg::Constraints& constraints)
{
  return constraints.joint_constraints.empty() && constraints.position_constraints.empty() &&
         constraints.orientation_constraints.empty() && constraints.visibility_constraints.empty();
}
}  // namespace

bool NTFieldsPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                        const rclcpp::Node::SharedPtr& node,
                                        const std::string& parameter_namespace)
{
  model_ = model;
  node_ = node;
  parameter_namespace_ = parameter_namespace;

  if (!model_ || !node_)
  {
    return false;
  }

  const auto scoped_name = [this](const std::string& param_name) {
    return parameter_namespace_.empty() ? param_name : parameter_namespace_ + "." + param_name;
  };

  const auto load_string_param = [this, &scoped_name](const std::string& param_name, std::string& value) {
    const std::string full_name = scoped_name(param_name);
    if (node_->has_parameter(full_name))
    {
      node_->get_parameter(full_name, value);
    }
    else
    {
      value = node_->declare_parameter<std::string>(full_name, value);
    }
  };

  const auto load_double_param = [this, &scoped_name](const std::string& param_name, double& value) {
    const std::string full_name = scoped_name(param_name);
    if (node_->has_parameter(full_name))
    {
      node_->get_parameter(full_name, value);
    }
    else
    {
      value = node_->declare_parameter<double>(full_name, value);
    }
  };

  const auto load_int_param = [this, &scoped_name](const std::string& param_name, std::size_t& value) {
    const std::string full_name = scoped_name(param_name);
    int64_t int_value = static_cast<int64_t>(value);
    if (node_->has_parameter(full_name))
    {
      node_->get_parameter(full_name, int_value);
    }
    else
    {
      int_value = node_->declare_parameter<int64_t>(full_name, int_value);
    }
    value = static_cast<std::size_t>(std::max<int64_t>(1, int_value));
  };

  load_string_param("group_name", group_name_);
  load_string_param("planner_url", planner_url_);
  load_double_param("request_timeout", request_timeout_);
  load_double_param("default_dt", default_dt_);
  load_int_param("interpolation_steps", interpolation_steps_);

  RCLCPP_INFO(node_->get_logger(),
              "Initialized NTFields planner manager for group '%s' with planner_url='%s'.",
              group_name_.c_str(), planner_url_.c_str());
  return true;
}

std::string NTFieldsPlannerManager::getDescription() const
{
  return "NTFields Joint Interpolation Planner";
}

void NTFieldsPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back(planner_id_);
}

planning_interface::PlanningContextPtr
NTFieldsPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                           const planning_interface::MotionPlanRequest& req,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

  if (!model_)
  {
    if (node_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Planner manager has not been initialized with a robot model.");
    }
    return {};
  }

  if (!canServiceRequest(req))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return {};
  }

  const moveit::core::JointModelGroup* joint_model_group = model_->getJointModelGroup(req.group_name);
  if (!joint_model_group)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return {};
  }

  auto context = std::make_shared<NTFieldsPlanningContext>(planner_id_, req.group_name, model_, joint_model_group,
                                                           node_, planner_url_, request_timeout_, default_dt_,
                                                           interpolation_steps_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool NTFieldsPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
{
  if (req.group_name != group_name_)
  {
    return false;
  }

  if (!supportsOnlyJointGoals(req))
  {
    return false;
  }

  if (req.goal_constraints.size() != 1)
  {
    return false;
  }

  if (req.goal_constraints.front().joint_constraints.empty())
  {
    return false;
  }

  return true;
}

void NTFieldsPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs)
{
  config_settings_ = pcs;
}

bool NTFieldsPlannerManager::supportsOnlyJointGoals(const planning_interface::MotionPlanRequest& req) const
{
  if (!constraintsEmpty(req.path_constraints))
  {
    return false;
  }

  if (!req.trajectory_constraints.constraints.empty())
  {
    return false;
  }

  for (const auto& goal : req.goal_constraints)
  {
    if (goal.joint_constraints.empty())
    {
      return false;
    }

    if (!goal.position_constraints.empty() || !goal.orientation_constraints.empty() ||
        !goal.visibility_constraints.empty())
    {
      return false;
    }
  }

  return true;
}

}  // namespace moveit_ntfields_planner

PLUGINLIB_EXPORT_CLASS(moveit_ntfields_planner::NTFieldsPlannerManager, planning_interface::PlannerManager)
