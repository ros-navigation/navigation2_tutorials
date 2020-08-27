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

#ifndef NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_
#define NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "nav2_sms_recovery/action/sms_recovery.hpp"
#include "nav2_sms_recovery/twilio.hpp"

namespace nav2_recoveries
{
using Action = nav2_sms_recovery::action::SmsRecovery;

class SMSRecovery : public Recovery<Action>
{
public:
  SMSRecovery();
  ~SMSRecovery();

  Status onRun(const std::shared_ptr<const Action::Goal> command) override;

  Status onCycleUpdate() override;
protected:
  std::string _account_sid;
  std::string _auth_token;
  std::string _from_number;
  std::string _to_number;
  std::shared_ptr<twilio::Twilio> _twilio;
};

}  // namespace nav2_recoveries

#endif  // NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_
