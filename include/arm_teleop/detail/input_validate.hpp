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

#include <arm_teleop/detail/input_command.hpp>
#include <arm_teleop/detail/logger.hpp>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <string_view>
#include <variant>

namespace arm_teleop::detail {

/**
 * @brief      This visitor validates the input messages are valid.
 */
class InputValidate : public InputVisitor {
 public:
  // TODO(tylerjw): create result type instead of optional<string>
  template <typename T>
  using Validate = std::function<std::optional<std::string>(const T&)>;

 private:
  std::shared_ptr<InputVisitor> next_ = nullptr;
  Logger logger_ = Logger("arm_teleop.input_check_valid_scaled");
  Validate<geometry_msgs::msg::TwistStamped> validateTwistStamped;
  Validate<control_msgs::msg::JointJog> validateJointJog;

  template <typename T>
  void operatorImpl(const T& command, const Validate<T>& validate) {
    if (auto error = validate(command)) {
      logger_.error(*error + "  Dropping message.");
      return;
    }

    // All is good, visit the next one
    std::visit(*next_, InputCommand{command});
  }

 public:
  /**
   * @brief      Constructs visitor for validating input commands.  Invalid
   * messages are dropped.
   *
   * @param[in]  command_in_type  The command in type (`scaled` or
   * `speed_units`)
   * @param[in]  next             The next visitor
   */
  InputValidate(std::string_view command_in_type,
                std::shared_ptr<InputVisitor> next);
  ~InputValidate() override = default;
  void operator()(const geometry_msgs::msg::TwistStamped& command) override {
    operatorImpl(command, validateTwistStamped);
  }

  void operator()(const control_msgs::msg::JointJog& command) override {
    operatorImpl(command, validateJointJog);
  }
};

}  // namespace arm_teleop::detail
