// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_sms_recovery/sms_recovery.hpp"

namespace nav2_sms_recovery
{

SMSRecovery::SMSRecovery()
: Recovery<Action>()
{
}

SMSRecovery::~SMSRecovery()
{
}

void SMSRecovery::onConfigure()
{
  auto node = node_.lock();
  node->declare_parameter("account_sid");
  _account_sid = node->get_parameter("account_sid").as_string();
  node->declare_parameter("auth_token");
  _auth_token = node->get_parameter("auth_token").as_string();
  node->declare_parameter("from_number");
  _from_number = node->get_parameter("from_number").as_string();
  node->declare_parameter("to_number");
  _to_number = node->get_parameter("to_number").as_string();
  _twilio = std::make_shared<twilio::Twilio>(_account_sid, _auth_token);
}

Status SMSRecovery::onRun(const std::shared_ptr<const Action::Goal> command)
{
 	auto node = node_.lock();   
  std::string response;  
  bool message_success = _twilio->send_message(
    _to_number, 
    _from_number, 
    command->message,
    response,
    "",
    false);

  if (!message_success) {
    RCLCPP_INFO(node->get_logger(), "SMS send failed.");
    return Status::FAILED;
  }

  RCLCPP_INFO(node->get_logger(), "SMS sent successfully!");
  return Status::SUCCEEDED;
}

Status SMSRecovery::onCycleUpdate()
{
  return Status::SUCCEEDED;
}

}  // namespace nav2_sms_recovery

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_sms_recovery::SMSRecovery, nav2_core::Recovery)
