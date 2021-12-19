// Copyright 2021 PickNik Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace arm_teleop::detail {
/**
 * @brief      Logger that calls rclcpp macros.
 */
class Logger {
  rclcpp::Logger logger_ = rclcpp::get_logger("arm_teleop");

 public:
  Logger(const std::string& log_namespace)
      : logger_{rclcpp::get_logger(log_namespace)} {}

// These logging macros don't play nicely with this compiler warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"

  void fatal(const std::string& what) const {
    // These are all NOLINT because clang-tidy doesn't like the c-code they
    // exapnd into.
    RCLCPP_FATAL(logger_, what.c_str());  // NOLINT
  }

  void error(const std::string& what) const {
    // These are all NOLINT because clang-tidy doesn't like the c-code they
    // exapnd into.
    RCLCPP_ERROR(logger_, what.c_str());  // NOLINT
  }

  void warn(const std::string& what) const {
    // These are all NOLINT because clang-tidy doesn't like the c-code they
    // exapnd into.
    RCLCPP_WARN(logger_, what.c_str());  // NOLINT
  }

  void info(const std::string& what) const {
    // These are all NOLINT because clang-tidy doesn't like the c-code they
    // exapnd into.
    RCLCPP_INFO(logger_, what.c_str());  // NOLINT
  }

  void debug(const std::string& what) const {
    // These are all NOLINT because clang-tidy doesn't like the c-code they
    // exapnd into.
    RCLCPP_DEBUG(logger_, what.c_str());  // NOLINT
  }

#pragma GCC diagnostic pop
};

}  // namespace arm_teleop::detail
