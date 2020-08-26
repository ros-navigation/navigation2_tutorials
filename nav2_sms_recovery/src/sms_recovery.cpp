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

#include "sms_recovery.hpp"
#include "twilio.hpp"

namespace nav2_recoveries
{

SMSRecovery::SMSRecovery()
: Recovery<WaitAction>(),
  feedback_(std::make_shared<WaitAction::Feedback>())
{
  // TODO get param
  _account_sid = "";
  _auth_token = "";
  _from_number = "";
  _to_number = "";
  _twilio = std::make_shared<twilio::Twilio>(_account_sid, _auth_token);
}

SMSRecovery::~SMSRecovery()
{
}

Status SMSRecovery::onRun(const std::shared_ptr<const WaitAction::Goal> command)
{
  std::string response;  
  bool message_success = _twilio->send_message(
    _to_number, 
    _from_number, 
    goal->message,
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
