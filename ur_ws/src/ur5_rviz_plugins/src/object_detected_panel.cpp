
#include "ur5_rviz_plugins/object_detected_panel.hpp"

#include <chrono>
#include <functional>

#include <QFont>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace ur5_rviz_plugins
{

namespace
{
constexpr char kObjectDetectedTopic[] = "/gripper/robotiq_hande_telemetry/object_detected";
constexpr int kRefreshMs = 100;       // 10 Hz de refresco visual
constexpr int64_t kStaleNs = 2'000'000'000;  // 2 s sin mensajes => sin datos
}  // namespace

ObjectDetectedPanel::ObjectDetectedPanel(QWidget * parent)
: rviz_common::Panel(parent),
  indicator_(new QLabel("SIN DATOS", this)),
  timer_(new QTimer(this))
{
  indicator_->setAlignment(Qt::AlignCenter);
  indicator_->setMinimumHeight(80);
  QFont font = indicator_->font();
  font.setPointSize(18);
  font.setBold(true);
  indicator_->setFont(font);
  indicator_->setAutoFillBackground(true);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Deteccion de objeto del gripper:", this));
  layout->addWidget(indicator_);
  setLayout(layout);

  connect(timer_, &QTimer::timeout, this, &ObjectDetectedPanel::refresh);
  refresh();
}

void ObjectDetectedPanel::onInitialize()
{
  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    indicator_->setText("SIN NODO ROS");
    return;
  }

  node_ = rviz_ros_node->get_raw_node();
  object_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kObjectDetectedTopic,
    rclcpp::QoS(10),
    std::bind(&ObjectDetectedPanel::onObjectDetected, this, std::placeholders::_1));

  timer_->start(kRefreshMs);
}

void ObjectDetectedPanel::onObjectDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
  detected_.store(msg->data, std::memory_order_relaxed);
  received_.store(true, std::memory_order_relaxed);
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  last_msg_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count(),
    std::memory_order_relaxed);
}

void ObjectDetectedPanel::refresh()
{
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const bool stale =
    !received_.load(std::memory_order_relaxed) ||
    (now - last_msg_ns_.load(std::memory_order_relaxed)) > kStaleNs;

  if (stale) {
    indicator_->setText("SIN DATOS");
    indicator_->setStyleSheet(
      "QLabel { background-color: #9e9e9e; color: white; border-radius: 6px; }");
    return;
  }

  if (detected_.load(std::memory_order_relaxed)) {
    indicator_->setText("OBJETO DETECTADO");
    indicator_->setStyleSheet(
      "QLabel { background-color: #2e7d32; color: white; border-radius: 6px; }");
  } else {
    indicator_->setText("SIN OBJETO");
    indicator_->setStyleSheet(
      "QLabel { background-color: #c62828; color: white; border-radius: 6px; }");
  }
}

}  // namespace ur5_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ur5_rviz_plugins::ObjectDetectedPanel, rviz_common::Panel)
