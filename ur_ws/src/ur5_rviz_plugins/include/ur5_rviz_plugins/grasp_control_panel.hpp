#ifndef UR5_RVIZ_PLUGINS__GRASP_CONTROL_PANEL_HPP_
#define UR5_RVIZ_PLUGINS__GRASP_CONTROL_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur5_manipulation/srv/set_vlm_instruction.hpp"

namespace ur5_rviz_plugins
{

class GraspControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GraspControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected Q_SLOTS:
  void prepareGraspCycle();
  void executeCachedGraspCycle();
  void scanObject();
  void setVlmInstruction();
  void prepareVlmGraspCycle();

private:
  using Trigger = std_srvs::srv::Trigger;
  using TriggerClient = rclcpp::Client<Trigger>;
  using SetVlmInstruction = ur5_manipulation::srv::SetVlmInstruction;
  using SetVlmInstructionClient = rclcpp::Client<SetVlmInstruction>;

  void callTriggerService(
    const std::string & service_name,
    const QString & pending_text,
    QPushButton * button);
  void setStatus(const QString & text, bool error = false);
  void setButtonsEnabled(bool enabled);

  rclcpp::Node::SharedPtr node_;
  TriggerClient::SharedPtr prepare_client_;
  TriggerClient::SharedPtr execute_client_;
  TriggerClient::SharedPtr scan_object_client_;
  TriggerClient::SharedPtr prepare_vlm_client_;
  SetVlmInstructionClient::SharedPtr set_instruction_client_;

  QLineEdit * instruction_edit_;
  QPushButton * set_instruction_button_;
  QPushButton * prepare_vlm_button_;
  QPushButton * prepare_button_;
  QPushButton * execute_button_;
  QPushButton * scan_object_button_;
  QLabel * status_label_;
};

}  // namespace ur5_rviz_plugins

#endif  // UR5_RVIZ_PLUGINS__GRASP_CONTROL_PANEL_HPP_
