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

#include <arm_teleop/detail/input_check_valid.hpp>
#include <cmath>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <variant>

#include "arm_teleop/detail/input_command.hpp"
#include "arm_teleop/detail/logger.hpp"

namespace arm_teleop::detail {

using std::fabs;
using std::isnan;

namespace {
bool containsNaN(const geometry_msgs::msg::TwistStamped& command) {
  return (isnan(command.twist.linear.x) || isnan(command.twist.linear.y) ||
          isnan(command.twist.linear.z) || isnan(command.twist.angular.x) ||
          isnan(command.twist.angular.y) || isnan(command.twist.angular.z));
}

bool containsNaN(const control_msgs::msg::JointJog& command) {
  return std::any_of(command.velocities.begin(), command.velocities.end(),
                     [](auto velocity) { return isnan(velocity); });
}

bool outsideScaledRange(const geometry_msgs::msg::TwistStamped& command) {
  return ((fabs(command.twist.linear.x) > 1) ||
          (fabs(command.twist.linear.y) > 1) ||
          (fabs(command.twist.linear.z) > 1) ||
          (fabs(command.twist.angular.x) > 1) ||
          (fabs(command.twist.angular.y) > 1) ||
          (fabs(command.twist.angular.z) > 1));
}
}  // namespace

void InputCheckValidScaled::operator()(
    const geometry_msgs::msg::TwistStamped& command) {
  if (containsNaN(command)) {
    logger_.warn("nan in incoming command. Skipping this datapoint.");
    return;
  }

  // Incoming commands should be in the range [-1:1], check for |delta|>1
  if (outsideScaledRange(command)) {
    logger_.warn(
        "Component of incoming command is outside the range [-1:1]. Skipping "
        "this datapoint.");
    return;
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{command});
}

void InputCheckValidSpeedUnits::operator()(
    const geometry_msgs::msg::TwistStamped& command) {
  if (containsNaN(command)) {
    logger_.warn("nan in incoming command. Skipping this datapoint.");
    return;
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{command});
}

// TODO(tylerjw): Deduplicate this code. Operator overloads don't work with
// inheritance.
void InputCheckValidScaled::operator()(
    const control_msgs::msg::JointJog& command) {
  if (containsNaN(command)) {
    logger_.warn("nan in incoming command. Skipping this datapoint.");
    return;
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{command});
}

void InputCheckValidSpeedUnits::operator()(
    const control_msgs::msg::JointJog& command) {
  if (containsNaN(command)) {
    logger_.warn("nan in incoming command. Skipping this datapoint.");
    return;
  }

  // All is good, visit the next one
  std::visit(*next_, InputCommand{command});
}

std::shared_ptr<InputVisitor> makeInputCheckValid(
    const rclcpp::Node::SharedPtr& node, std::string_view command_in_type,
    const std::shared_ptr<InputVisitor>& next) {
  if (command_in_type == "scaled") {
    return std::make_shared<InputCheckValidScaled>(node, next);
  }
  if (command_in_type == "speed_units") {
    return std::make_shared<InputCheckValidSpeedUnits>(node, next);
  }
  std::stringstream ss;
  ss << '`' << command_in_type << "` must be either `scaled` or `speed_units`";
  Logger("arm_teleop.input_check_valid").fatal(ss.str());
  throw std::runtime_error(ss.str());
}

}  // namespace arm_teleop::detail
