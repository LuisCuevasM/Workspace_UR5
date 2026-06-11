#ifndef UR5_RVIZ_PLUGINS__SKILL_CONTROL_PANEL_HPP_
#define UR5_RVIZ_PLUGINS__SKILL_CONTROL_PANEL_HPP_

#include <map>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "ur5_manipulation/srv/run_skill.hpp"

namespace ur5_rviz_plugins
{

// Panel para probar la capa de "robot skills" del UR5 (robot_skill_node).
// Cada boton llama a un servicio ur5_manipulation/srv/RunSkill bajo /robot.
class SkillControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SkillControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  using RunSkill = ur5_manipulation::srv::RunSkill;
  using RunSkillClient = rclcpp::Client<RunSkill>;

  QPushButton * addSkillButton(
    const QString & label,
    const std::string & service_name,
    const QString & pending_text);
  void callRunSkillService(
    const std::string & service_name,
    const QString & pending_text,
    QPushButton * button);
  void setStatus(const QString & text, bool error = false);
  void setButtonsEnabled(bool enabled);

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, RunSkillClient::SharedPtr> clients_;
  std::vector<QPushButton *> buttons_;
  QLineEdit * object_name_edit_;
  QLineEdit * target_location_edit_;
  QCheckBox * clear_cached_grasps_check_;
  QCheckBox * force_new_perception_check_;
  QSpinBox * attempt_number_spin_;
  QLabel * status_label_;
  QVBoxLayout * layout_;
};

}  // namespace ur5_rviz_plugins

#endif  // UR5_RVIZ_PLUGINS__SKILL_CONTROL_PANEL_HPP_
