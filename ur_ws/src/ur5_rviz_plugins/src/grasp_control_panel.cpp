#include "ur5_rviz_plugins/grasp_control_panel.hpp"

#include <functional>

#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace ur5_rviz_plugins
{

namespace
{
constexpr char kPrepareService[] = "/ur5/prepare_bt_grasp_cycle";
constexpr char kExecuteService[] = "/ur5/execute_cached_grasp_cycle";
constexpr char kScanObjectService[] = "/point_cloud_scanner/capture_scan";
}  // namespace

GraspControlPanel::GraspControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  prepare_button_(new QPushButton("Preparar grasp", this)),
  execute_button_(new QPushButton("Ejecutar grasp cached", this)),
  scan_object_button_(new QPushButton("Scan object", this)),
  status_label_(new QLabel("Esperando inicializacion de RViz...", this))
{
  prepare_button_->setToolTip("Llama /ur5/prepare_bt_grasp_cycle");
  execute_button_->setToolTip("Llama /ur5/execute_cached_grasp_cycle");
  scan_object_button_->setToolTip("Llama /point_cloud_scanner/capture_scan");
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(prepare_button_);
  layout->addWidget(execute_button_);
  layout->addWidget(scan_object_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  setButtonsEnabled(false);

  connect(prepare_button_, &QPushButton::clicked, this, &GraspControlPanel::prepareGraspCycle);
  connect(execute_button_, &QPushButton::clicked, this, &GraspControlPanel::executeCachedGraspCycle);
  connect(scan_object_button_, &QPushButton::clicked, this, &GraspControlPanel::scanObject);
}

void GraspControlPanel::onInitialize()
{
  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    setStatus("No se pudo obtener el nodo ROS de RViz.", true);
    return;
  }

  node_ = rviz_ros_node->get_raw_node();
  prepare_client_ = node_->create_client<Trigger>(kPrepareService);
  execute_client_ = node_->create_client<Trigger>(kExecuteService);
  scan_object_client_ = node_->create_client<Trigger>(kScanObjectService);

  setButtonsEnabled(true);
  setStatus("Listo para llamar los servicios del UR5.");
}

void GraspControlPanel::prepareGraspCycle()
{
  callTriggerService(kPrepareService, "Preparando ciclo de grasp...", prepare_button_);
}

void GraspControlPanel::executeCachedGraspCycle()
{
  callTriggerService(kExecuteService, "Ejecutando grasp cached...", execute_button_);
}

void GraspControlPanel::scanObject()
{
  callTriggerService(kScanObjectService, "Escaneando objeto...", scan_object_button_);
}

void GraspControlPanel::callTriggerService(
  const std::string & service_name,
  const QString & pending_text,
  QPushButton * button)
{
  if (!node_) {
    setStatus("El panel aun no tiene nodo ROS.", true);
    return;
  }

  TriggerClient::SharedPtr client =
    service_name == kPrepareService ? prepare_client_ :
    service_name == kExecuteService ? execute_client_ : scan_object_client_;

  if (!client || !client->service_is_ready()) {
    setStatus(QString("Servicio no disponible: %1").arg(QString::fromStdString(service_name)), true);
    return;
  }

  setButtonsEnabled(false);
  button->setEnabled(false);
  setStatus(pending_text);

  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
    request,
    [this, service_name](TriggerClient::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString::fromStdString(response->message);
      const QString service = QString::fromStdString(service_name);
      const bool success = response->success;

      QMetaObject::invokeMethod(
        this,
        [this, service, message, success]() {
          setButtonsEnabled(true);
          setStatus(
            QString("%1: %2%3")
            .arg(success ? "OK" : "Fallo")
            .arg(service)
            .arg(message.isEmpty() ? QString() : QString(" - %1").arg(message)),
            !success);
        },
        Qt::QueuedConnection);
    });
}

void GraspControlPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "QLabel { color: palette(text); }");
}

void GraspControlPanel::setButtonsEnabled(bool enabled)
{
  prepare_button_->setEnabled(enabled);
  execute_button_->setEnabled(enabled);
  scan_object_button_->setEnabled(enabled);
}

}  // namespace ur5_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ur5_rviz_plugins::GraspControlPanel, rviz_common::Panel)
