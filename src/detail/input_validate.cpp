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

#include <algorithm>
#include <arm_teleop/detail/input_validate.hpp>
#include <cmath>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "arm_teleop/detail/logger.hpp"

namespace arm_teleop::detail {

class InputVisitor;

using std::fabs;
using std::isnan;

namespace {

constexpr auto kNaNError = "NaN in incoming command.";
constexpr auto kRangeError =
    "Component of incoming command is outside the range [-1:1].";

std::optional<std::string> validateNoNaN(
    const geometry_msgs::msg::TwistStamped& command) {
  if (isnan(command.twist.linear.x) || isnan(command.twist.linear.y) ||
      isnan(command.twist.linear.z) || isnan(command.twist.angular.x) ||
      isnan(command.twist.angular.y) || isnan(command.twist.angular.z)) {
    return kNaNError;
  }
  return std::nullopt;
}

std::optional<std::string> validateNoNaN(
    const control_msgs::msg::JointJog& command) {
  if (std::any_of(command.velocities.begin(), command.velocities.end(),
                  [](auto velocity) { return isnan(velocity); })) {
    return kNaNError;
  }
  return std::nullopt;
}

std::optional<std::string> validateSclae(
    const geometry_msgs::msg::TwistStamped& command) {
  if ((fabs(command.twist.linear.x) > 1) ||
      (fabs(command.twist.linear.y) > 1) ||
      (fabs(command.twist.linear.z) > 1) ||
      (fabs(command.twist.angular.x) > 1) ||
      (fabs(command.twist.angular.y) > 1) ||
      (fabs(command.twist.angular.z) > 1)) {
    return kRangeError;
  }
  return std::nullopt;
}

InputValidate::Validate<geometry_msgs::msg::TwistStamped>
makeValidateTwistStamped(std::string_view command_in_type) {
  if (command_in_type == "scaled") {
    return [](const geometry_msgs::msg::TwistStamped& command)
               -> std::optional<std::string> {
      // truthyness is an error string
      if (auto error = validateNoNaN(command)) {
        return *error;
      }
      return validateSclae(command);
    };
  }
  if (command_in_type == "speed_units") {
    return [](const geometry_msgs::msg::TwistStamped& command)
               -> std::optional<std::string> { return validateNoNaN(command); };
  }
  std::stringstream ss;
  ss << '`' << command_in_type << "` must be either `scaled` or `speed_units`";
  Logger("arm_teleop.input_check_valid").fatal(ss.str());
  throw std::runtime_error(ss.str());
}
}  // namespace

InputValidate::InputValidate(std::string_view command_in_type,
                             std::shared_ptr<InputVisitor> next)
    : next_{std::move(next)},
      validateTwistStamped{makeValidateTwistStamped(command_in_type)},
      validateJointJog{[](const control_msgs::msg::JointJog& command) {
        return validateNoNaN(command);
      }} {
  if (!static_cast<bool>(next_)) {
    throw std::runtime_error("Next Visitor was not valid");
  }
}
}  // namespace arm_teleop::detail
