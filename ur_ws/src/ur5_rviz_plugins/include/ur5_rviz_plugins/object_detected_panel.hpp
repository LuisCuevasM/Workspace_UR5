#ifndef UR5_RVIZ_PLUGINS__OBJECT_DETECTED_PANEL_HPP_
#define UR5_RVIZ_PLUGINS__OBJECT_DETECTED_PANEL_HPP_

#include <atomic>
#include <memory>

#include <QLabel>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/bool.hpp"

class QTimer;

namespace ur5_rviz_plugins
{

// Panel dedicado que muestra en tiempo real el estado del sensor de deteccion
// de objeto del gripper Hand-E. Refresca con un QTimer en el hilo de Qt, asi
// que el indicador cambia de color aunque el callback ROS corra en otro hilo.
class ObjectDetectedPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ObjectDetectedPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected Q_SLOTS:
  void refresh();

private:
  void onObjectDetected(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_detected_sub_;

  QLabel * indicator_;
  QTimer * timer_;

  std::atomic<bool> detected_{false};
  std::atomic<bool> received_{false};
  std::atomic<int64_t> last_msg_ns_{0};
};

}  // namespace ur5_rviz_plugins

#endif  // UR5_RVIZ_PLUGINS__OBJECT_DETECTED_PANEL_HPP_
