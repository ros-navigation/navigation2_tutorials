// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_sms_recovery/sms_recovery.hpp"

namespace nav2_recoveries
{

SMSRecovery::SMSRecovery()
: Recovery<Action>()
{
  node_->declare_parameter("account_sid");
  _account_sid = node_->get_parameter("account_sid").as_string();
  node_->declare_parameter("auth_token");
  _auth_token = node_->get_parameter("auth_token").as_string();
  node_->declare_parameter("from_number");
  _from_number = node_->get_parameter("from_number").as_string();
  node_->declare_parameter("to_number");
  _to_number = node_->get_parameter("to_number").as_string();
  _twilio = std::make_shared<twilio::Twilio>(_account_sid, _auth_token);
}

SMSRecovery::~SMSRecovery()
{
}

Status SMSRecovery::onRun(const std::shared_ptr<const Action::Goal> command)
{
  std::string response;  
  bool message_success = _twilio->send_message(
    _to_number, 
    _from_number, 
    command->message,
    response,
    "",
    false);

  if (!message_success) {
    RCLCPP_INFO(node_->get_logger(), "Message send failed.");
    return Status::FAILED;
  }

  RCLCPP_INFO(node_->get_logger(), "SMS sent successfully!");
  return Status::SUCCEEDED;
}

Status SMSRecovery::onCycleUpdate()
{
  return Status::SUCCEEDED;
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::SMSRecovery, nav2_core::Recovery)
