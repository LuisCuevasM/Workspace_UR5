#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GraspToMoveItNode : public rclcpp::Node
{
public:
  GraspToMoveItNode()
  : Node("grasp_to_moveit_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    target_frame_("base_link"),
    has_pending_plan_(false)
  {
    base_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/graspgen/best_grasp_base", 10);
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10);

    grasp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/graspgen/best_grasp", 10,
      std::bind(&GraspToMoveItNode::graspCallback, this, std::placeholders::_1));

    execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/grasp2moveit/execute_plan",
      std::bind(
        &GraspToMoveItNode::executePlanCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "GraspToMoveItNode creado, inicializando MoveIt...");
  }

  void initMoveGroup()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_arm");
    move_group_->setEndEffectorLink("tool0");
    move_group_->setPoseReferenceFrame(target_frame_);
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.15);

    RCLCPP_INFO(
      get_logger(),
      "GraspToMoveItNode listo. Planifica con /graspgen/best_grasp y ejecuta con "
      "ros2 service call /grasp2moveit/execute_plan std_srvs/srv/Trigger {}");
  }

private:
  void graspCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    try {
      if (!move_group_) {
        RCLCPP_ERROR(get_logger(), "MoveGroupInterface no esta inicializado");
        return;
      }

      geometry_msgs::msg::PoseStamped pose_base = tf_buffer_.transform(
        *msg, target_frame_, tf2::durationFromSec(1.0));

      pose_base.pose.position.z += -0.197;
      base_pub_->publish(pose_base);

      const auto current_pose = move_group_->getCurrentPose("tool0");
      RCLCPP_INFO(
        get_logger(),
        "Objetivo en %s | pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
        target_frame_.c_str(),
        pose_base.pose.position.x,
        pose_base.pose.position.y,
        pose_base.pose.position.z,
        pose_base.pose.orientation.x,
        pose_base.pose.orientation.y,
        pose_base.pose.orientation.z,
        pose_base.pose.orientation.w);
      RCLCPP_INFO(
        get_logger(),
        "Pose actual tool0 | pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z,
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);

      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(pose_base.pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success =
        move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

      if (!success) {
        move_group_->clearPoseTargets();
        RCLCPP_WARN(
          get_logger(),
          "No se encontro plan con pose exacta. Intentando IK aproximada...");

        const bool target_ok = move_group_->setApproximateJointValueTarget(
          pose_base.pose, "tool0");

        if (target_ok) {
          move_group_->setStartStateToCurrentState();
          success = move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        } else {
          RCLCPP_WARN(get_logger(), "No se pudo fijar target aproximado para tool0");
        }
      }

      if (!success) {
        has_pending_plan_ = false;
        move_group_->clearPoseTargets();
        RCLCPP_ERROR(get_logger(), "No plan para el grasp recibido");
        return;
      }

      pending_plan_ = plan;
      pending_target_ = pose_base;
      has_pending_plan_ = true;
      publishDisplayTrajectory();
      move_group_->clearPoseTargets();

      RCLCPP_INFO(
        get_logger(),
        "Trayectoria planificada correctamente. Ejecuta con "
        "ros2 service call /grasp2moveit/execute_plan std_srvs/srv/Trigger {}");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Transform TF fallo: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error general: %s", ex.what());
    }
  }

  void publishDisplayTrajectory()
  {
    if (!has_pending_plan_) {
      return;
    }

    moveit_msgs::msg::DisplayTrajectory msg;
    msg.model_id = "ur5_full";
    msg.trajectory_start = pending_plan_.start_state_;
    msg.trajectory.push_back(pending_plan_.trajectory_);
    display_pub_->publish(msg);
  }

  void executePlanCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!has_pending_plan_) {
      response->success = false;
      response->message = "No hay trayectoria planificada pendiente";
      return;
    }

    try {
      const bool ok = move_group_->execute(pending_plan_) == moveit::core::MoveItErrorCode::SUCCESS;
      move_group_->stop();

      if (ok) {
        has_pending_plan_ = false;
        response->success = true;
        response->message = "Trayectoria ejecutada correctamente";
      } else {
        response->success = false;
        response->message = "La ejecucion de la trayectoria fallo";
      }
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = std::string("Error ejecutando trayectoria: ") + ex.what();
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  moveit::planning_interface::MoveGroupInterface::Plan pending_plan_;
  geometry_msgs::msg::PoseStamped pending_target_;
  bool has_pending_plan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GraspToMoveItNode>();
  node->initMoveGroup();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
