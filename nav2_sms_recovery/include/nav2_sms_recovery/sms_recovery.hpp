// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_
#define NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "nav2_sms_recovery/action/sms_recovery.hpp"
#include "nav2_sms_recovery/twilio.hpp"

namespace nav2_sms_recovery
{

using namespace nav2_recoveries;  // NOLINT
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

}  // namespace nav2_sms_recovery

#endif  // NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_
