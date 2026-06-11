#include "ur5_rviz_plugins/skill_control_panel.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMetaObject>
#include <QSpinBox>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace ur5_rviz_plugins
{

namespace
{
constexpr char kGraspService[] = "/robot/grasp";
constexpr char kPickService[] = "/robot/pick";
constexpr char kPlaceService[] = "/robot/place";
constexpr char kPickPlaceService[] = "/robot/pick_place";
constexpr char kPushService[] = "/robot/push";
constexpr char kHandoverService[] = "/robot/handover";
}  // namespace

SkillControlPanel::SkillControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  object_name_edit_(new QLineEdit(this)),
  target_location_edit_(new QLineEdit(this)),
  clear_cached_grasps_check_(new QCheckBox("Limpiar grasps cacheados", this)),
  force_new_perception_check_(new QCheckBox("Forzar nueva percepcion", this)),
  attempt_number_spin_(new QSpinBox(this)),
  status_label_(new QLabel("Esperando inicializacion de RViz...", this)),
  layout_(new QVBoxLayout(this))
{
  status_label_->setWordWrap(true);
  object_name_edit_->setPlaceholderText("opcional");
  target_location_edit_->setPlaceholderText("requerido para place/pick_place");
  attempt_number_spin_->setMinimum(1);
  attempt_number_spin_->setMaximum(999);
  attempt_number_spin_->setValue(1);

  auto * form = new QFormLayout();
  form->addRow("Objeto:", object_name_edit_);
  form->addRow("Destino:", target_location_edit_);
  form->addRow("Intento:", attempt_number_spin_);

  layout_->addWidget(new QLabel("Robot skills (robot_skill_node):", this));
  layout_->addLayout(form);
  layout_->addWidget(clear_cached_grasps_check_);
  layout_->addWidget(force_new_perception_check_);
  addSkillButton("Grasp SAM2 directo", kGraspService, "Generando grasps con SAM2 + GraspGen...");
  addSkillButton("Pick (tomar y levantar)", kPickService, "Ejecutando pick...");
  addSkillButton("Place (depositar)", kPlaceService, "Ejecutando place...");
  addSkillButton("Pick + Place", kPickPlaceService, "Ejecutando pick + place...");
  addSkillButton("Push (por definir)", kPushService, "Ejecutando push...");
  addSkillButton("Handover (por definir)", kHandoverService, "Ejecutando handover...");
  layout_->addWidget(status_label_);
  layout_->addStretch();
  setLayout(layout_);

  setButtonsEnabled(false);
}

QPushButton * SkillControlPanel::addSkillButton(
  const QString & label,
  const std::string & service_name,
  const QString & pending_text)
{
  auto * button = new QPushButton(label, this);
  button->setToolTip(QString("Llama %1").arg(QString::fromStdString(service_name)));
  layout_->addWidget(button);
  buttons_.push_back(button);
  connect(
    button, &QPushButton::clicked, this,
    [this, service_name, pending_text, button]() {
      callRunSkillService(service_name, pending_text, button);
    });
  return button;
}

void SkillControlPanel::onInitialize()
{
  auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    setStatus("No se pudo obtener el nodo ROS de RViz.", true);
    return;
  }

  node_ = rviz_ros_node->get_raw_node();
  for (const char * service :
    {kGraspService, kPickService, kPlaceService, kPickPlaceService, kPushService,
      kHandoverService})
  {
    clients_[service] = node_->create_client<RunSkill>(service);
  }

  setButtonsEnabled(true);
  setStatus("Listo para llamar los skills del robot.");
}

void SkillControlPanel::callRunSkillService(
  const std::string & service_name,
  const QString & pending_text,
  QPushButton * button)
{
  if (!node_) {
    setStatus("El panel aun no tiene nodo ROS.", true);
    return;
  }

  auto it = clients_.find(service_name);
  if (it == clients_.end() || !it->second || !it->second->service_is_ready()) {
    setStatus(
      QString("Servicio no disponible: %1").arg(QString::fromStdString(service_name)), true);
    return;
  }

  setButtonsEnabled(false);
  button->setEnabled(false);
  setStatus(pending_text);

  auto request = std::make_shared<RunSkill::Request>();
  request->object_name = object_name_edit_->text().trimmed().toStdString();
  request->target_location = target_location_edit_->text().trimmed().toStdString();
  request->clear_cached_grasps = clear_cached_grasps_check_->isChecked();
  request->force_new_perception = force_new_perception_check_->isChecked();
  request->vlm_context_json = "";
  request->attempt_number = attempt_number_spin_->value();

  it->second->async_send_request(
    request,
    [this, service_name](RunSkillClient::SharedFuture future) {
      const auto response = future.get();
      const QString service = QString::fromStdString(service_name);
      const bool success = response->success;
      const QString stage = QString::fromStdString(response->stage);
      const QString failure_type = QString::fromStdString(response->failure_type);
      const QString grasp_id = QString::fromStdString(response->grasp_id);
      const QString mask_id = QString::fromStdString(response->mask_id);

      QMetaObject::invokeMethod(
        this,
        [this, service, success, stage, failure_type, grasp_id, mask_id]() {
          setButtonsEnabled(true);
          QStringList parts;
          if (!stage.isEmpty()) {
            parts << QString("stage=%1").arg(stage);
          }
          if (!failure_type.isEmpty()) {
            parts << QString("failure=%1").arg(failure_type);
          }
          if (!grasp_id.isEmpty()) {
            parts << QString("grasp=%1").arg(grasp_id);
          }
          if (!mask_id.isEmpty()) {
            parts << QString("mask=%1").arg(mask_id);
          }
          setStatus(
            QString("%1: %2%3")
            .arg(success ? "OK" : "Fallo")
            .arg(service)
            .arg(parts.isEmpty() ? QString() : QString(" - %1").arg(parts.join(", "))),
            !success);
        },
        Qt::QueuedConnection);
    });
}

void SkillControlPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(
    error ? "QLabel { color: #b00020; }" : "QLabel { color: palette(text); }");
}

void SkillControlPanel::setButtonsEnabled(bool enabled)
{
  for (auto * button : buttons_) {
    button->setEnabled(enabled);
  }
}

}  // namespace ur5_rviz_plugins

PLUGINLIB_EXPORT_CLASS(ur5_rviz_plugins::SkillControlPanel, rviz_common::Panel)
