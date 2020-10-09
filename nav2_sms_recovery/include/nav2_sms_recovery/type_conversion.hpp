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

#ifndef NAV2_SMS_RECOVERY__TYPE_CONVERSION_HPP_
#define NAV2_SMS_RECOVERY__TYPE_CONVERSION_HPP_

#pragma once

#include <locale>
#include <codecvt>
#include <string>

namespace nav2_sms_recovery {
// Given a UTF-8 encoded string return a new UCS-2 string.
inline std::u16string
utf8_to_ucs2(std::string const& input)
{
   std::wstring_convert <std::codecvt_utf8 <char16_t>, char16_t> convert;

   try {
      return(convert.from_bytes(input));
   } catch (const std::range_error& e) {
      throw std::range_error(
               "Failed UCS-2 conversion of message body.  Check all "
               "characters are valid GSM-7, GSM 8-bit text, or UCS-2 "
               "characters."
               );
   }
}

// Given a UCS-2 string return a new UTF-8 encoded string.
inline std::string
ucs2_to_utf8(std::u16string const& input)
{
   std::wstring_convert <std::codecvt_utf8 <char16_t>, char16_t> convert;

   return(convert.to_bytes(input));
}
}  // namespace nav2_sms_recovery

#endif  // NAV2_SMS_RECOVERY__TYPE_CONVERSION_HPP_
