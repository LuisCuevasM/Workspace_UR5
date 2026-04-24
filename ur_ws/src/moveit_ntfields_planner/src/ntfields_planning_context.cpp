#include <moveit_ntfields_planner/ntfields_planning_context.hpp>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <unordered_map>

#include <curl/curl.h>
#include <json/json.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_ntfields_planner
{

namespace
{
size_t writeCurlResponse(void* contents, size_t size, size_t nmemb, void* userp)
{
  const size_t total_size = size * nmemb;
  auto* buffer = static_cast<std::string*>(userp);
  buffer->append(static_cast<const char*>(contents), total_size);
  return total_size;
}
}  // namespace

NTFieldsPlanningContext::NTFieldsPlanningContext(const std::string& name, const std::string& group,
                                                 const moveit::core::RobotModelConstPtr& model,
                                                 const moveit::core::JointModelGroup* joint_model_group,
                                                 const rclcpp::Node::SharedPtr& node, std::string planner_url,
                                                 double request_timeout, double default_dt,
                                                 std::size_t interpolation_steps)
  : planning_interface::PlanningContext(name, group)
  , model_(model)
  , joint_model_group_(joint_model_group)
  , node_(node)
  , planner_url_(std::move(planner_url))
  , request_timeout_(request_timeout)
  , default_dt_(default_dt)
  , interpolation_steps_(std::max<std::size_t>(interpolation_steps, 1))
{
}

bool NTFieldsPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  terminate_requested_ = false;

  if (!planning_scene_ || !model_ || !joint_model_group_)
  {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();

  moveit::core::RobotState start_state = planning_scene_->getCurrentState();
  if (!request_.start_state.joint_state.name.empty() || request_.start_state.is_diff)
  {
    if (!moveit::core::robotStateMsgToRobotState(request_.start_state, start_state, true))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse request start_state into a RobotState.");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return false;
    }
  }

  std::vector<double> q_start;
  std::vector<double> q_goal;
  if (!extractStart(start_state, q_start) || !extractGoal(q_goal))
  {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  std::vector<std::vector<double>> waypoints;
  if (!requestRemotePlan(q_start, q_goal, waypoints))
  {
    RCLCPP_WARN(node_->get_logger(),
                "Falling back to local joint interpolation because the remote NTFields planner request failed.");
    if (!buildInterpolatedPath(q_start, q_goal, waypoints))
    {
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }
  }

  robot_trajectory::RobotTrajectoryPtr trajectory;
  if (!buildTrajectory(waypoints, trajectory))
  {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  const auto ended_at = std::chrono::steady_clock::now();
  res.trajectory_ = trajectory;
  res.planning_time_ = std::chrono::duration<double>(ended_at - started_at).count();
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool NTFieldsPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  const bool solved = solve(simple_res);

  res.trajectory_.clear();
  res.description_.clear();
  res.processing_time_.clear();
  res.error_code_ = simple_res.error_code_;

  if (!solved)
  {
    return false;
  }

  res.trajectory_.push_back(simple_res.trajectory_);
  res.description_.push_back("plan");
  res.processing_time_.push_back(simple_res.planning_time_);
  return true;
}

bool NTFieldsPlanningContext::terminate()
{
  terminate_requested_ = true;
  return true;
}

void NTFieldsPlanningContext::clear()
{
  terminate_requested_ = false;
}

bool NTFieldsPlanningContext::extractStart(const moveit::core::RobotState& seed_state, std::vector<double>& q_start) const
{
  q_start.clear();
  seed_state.copyJointGroupPositions(joint_model_group_, q_start);
  return !q_start.empty();
}

bool NTFieldsPlanningContext::extractGoal(std::vector<double>& q_goal) const
{
  if (request_.goal_constraints.empty())
  {
    return false;
  }

  const auto& goal_constraints = request_.goal_constraints.front().joint_constraints;
  if (goal_constraints.empty())
  {
    return false;
  }

  std::unordered_map<std::string, double> goal_by_joint;
  goal_by_joint.reserve(goal_constraints.size());
  for (const auto& joint_constraint : goal_constraints)
  {
    goal_by_joint[joint_constraint.joint_name] = joint_constraint.position;
  }

  const std::vector<std::string>& variable_names = joint_model_group_->getVariableNames();
  q_goal.clear();
  q_goal.reserve(variable_names.size());

  for (const auto& variable_name : variable_names)
  {
    const auto it = goal_by_joint.find(variable_name);
    if (it == goal_by_joint.end())
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint goal is missing variable '%s'.", variable_name.c_str());
      return false;
    }
    q_goal.push_back(it->second);
  }

  return q_goal.size() == variable_names.size();
}

bool NTFieldsPlanningContext::requestRemotePlan(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                                                std::vector<std::vector<double>>& waypoints) const
{
  if (terminate_requested_)
  {
    return false;
  }

  Json::Value request_body(Json::objectValue);
  for (const double value : q_start)
  {
    request_body["q_start"].append(value);
  }
  for (const double value : q_goal)
  {
    request_body["q_goal"].append(value);
  }

  Json::StreamWriterBuilder writer_builder;
  const std::string request_json = Json::writeString(writer_builder, request_body);

  RCLCPP_INFO(node_->get_logger(), "Requesting remote NTFields plan at '%s' with timeout %.2f s.",
              planner_url_.c_str(), request_timeout_);

  CURL* curl = curl_easy_init();
  if (!curl)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize libcurl.");
    return false;
  }

  std::string response_body;
  curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, planner_url_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_json.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(request_json.size()));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS,
                   static_cast<long>(std::min(request_timeout_ * 1000.0, 2000.0)));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(request_timeout_ * 1000.0));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCurlResponse);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

  const CURLcode curl_result = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (curl_result != CURLE_OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "Remote planner request failed: %s", curl_easy_strerror(curl_result));
    return false;
  }

  if (http_code < 200 || http_code >= 300)
  {
    RCLCPP_ERROR(node_->get_logger(), "Remote planner returned HTTP status %ld with body: %s", http_code,
                 response_body.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Remote NTFields planner replied with HTTP %ld.", http_code);

  Json::CharReaderBuilder reader_builder;
  Json::Value response_json;
  std::string parse_errors;
  std::istringstream response_stream(response_body);
  if (!Json::parseFromStream(reader_builder, response_stream, &response_json, &parse_errors))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse remote planner JSON response: %s", parse_errors.c_str());
    return false;
  }

  if (!response_json.get("success", false).asBool())
  {
    RCLCPP_ERROR(node_->get_logger(), "Remote planner reported failure: %s", response_body.c_str());
    return false;
  }

  const Json::Value& trajectory_json = response_json["trajectory"];
  if (!trajectory_json.isArray() || trajectory_json.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Remote planner returned an empty or invalid trajectory.");
    return false;
  }

  const std::size_t expected_size = joint_model_group_->getVariableNames().size();
  waypoints.clear();
  waypoints.reserve(trajectory_json.size());

  for (const auto& waypoint_json : trajectory_json)
  {
    if (!waypoint_json.isArray() || waypoint_json.size() != expected_size)
    {
      RCLCPP_ERROR(node_->get_logger(), "Remote planner waypoint has invalid dimension.");
      waypoints.clear();
      return false;
    }

    std::vector<double> waypoint;
    waypoint.reserve(expected_size);
    for (const auto& joint_value_json : waypoint_json)
    {
      if (!joint_value_json.isNumeric())
      {
        RCLCPP_ERROR(node_->get_logger(), "Remote planner waypoint contains a non-numeric joint value.");
        waypoints.clear();
        return false;
      }
      waypoint.push_back(joint_value_json.asDouble());
    }
    waypoints.push_back(std::move(waypoint));
  }

  const std::string iterations =
      response_json.isMember("iterations") ? response_json["iterations"].toStyledString() : "n/a";
  const std::string planning_time =
      response_json.isMember("planning_time") ? response_json["planning_time"].toStyledString() : "n/a";

  RCLCPP_INFO(node_->get_logger(),
              "Remote NTFields planner succeeded with %zu waypoints | iterations=%s | planning_time=%s",
              waypoints.size(), iterations.c_str(), planning_time.c_str());

  return true;
}

bool NTFieldsPlanningContext::buildInterpolatedPath(const std::vector<double>& q_start, const std::vector<double>& q_goal,
                                                    std::vector<std::vector<double>>& waypoints) const
{
  if (q_start.size() != q_goal.size() || q_start.empty())
  {
    return false;
  }

  waypoints.clear();
  waypoints.reserve(interpolation_steps_ + 1);

  for (std::size_t step = 0; step <= interpolation_steps_; ++step)
  {
    if (terminate_requested_)
    {
      return false;
    }

    const double alpha = static_cast<double>(step) / static_cast<double>(interpolation_steps_);
    std::vector<double> waypoint(q_start.size(), 0.0);
    for (std::size_t i = 0; i < q_start.size(); ++i)
    {
      waypoint[i] = q_start[i] + alpha * (q_goal[i] - q_start[i]);
    }
    waypoints.push_back(std::move(waypoint));
  }

  return !waypoints.empty();
}

bool NTFieldsPlanningContext::buildTrajectory(const std::vector<std::vector<double>>& waypoints,
                                              robot_trajectory::RobotTrajectoryPtr& trajectory) const
{
  if (waypoints.empty())
  {
    return false;
  }

  trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(model_, joint_model_group_);
  moveit::core::RobotState waypoint_state(model_);
  waypoint_state.setToDefaultValues();

  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    waypoint_state.setJointGroupPositions(joint_model_group_, waypoints[i]);
    waypoint_state.update();
    trajectory->addSuffixWayPoint(waypoint_state, i == 0 ? 0.0 : default_dt_);
  }

  return true;
}

}  // namespace moveit_ntfields_planner
