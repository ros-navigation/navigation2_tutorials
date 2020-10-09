// MIT License
//
// Copyright (c) [year] [fullname]
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef NAV2_SMS_RECOVERY__TWILIO_HPP_
#define NAV2_SMS_RECOVERY__TWILIO_HPP_

#pragma once

#include <string>
#include "type_conversion.hpp"

namespace twilio {
class Twilio {
public:
   Twilio(std::string const& account_sid_in,
          std::string const& auth_token_in)
      : account_sid(account_sid_in)
      , auth_token(auth_token_in)
   {
   }

   // Nothing in destructor
   ~Twilio() = default;

   bool send_message(
      std::string const& to_number,
      std::string const& from_number,
      std::string const& message_body,
      std::string& response,
      std::string const& picture_url = "",
      bool verbose = false
      );

private:
   // Account SID and Auth Token come from the Twilio console.
   // See: https://twilio.com/console for more.

   // Used for the username of the auth header
   std::string const account_sid;
   // Used for the password of the auth header
   std::string const auth_token;

   // Portably ignore curl response
   static size_t _null_write(char *, size_t, size_t, void *);
   // Write curl response to a stringstream
   static size_t _stream_write(char *, size_t, size_t, void *);
};
}  //  end namespace twilio

#endif  //  NAV2_SMS_RECOVERY__TWILIO_HPP_
