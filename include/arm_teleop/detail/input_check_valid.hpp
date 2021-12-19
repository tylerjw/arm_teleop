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

#include <assert.h>

#include <arm_teleop/detail/input_command.hpp>
#include <arm_teleop/detail/logger.hpp>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <string_view>

namespace arm_teleop::detail {
/**
 * @brief      Validates that the input messages do not contain NAN and in the
 * case of TwistStamped that it is within the range [-1,1].
 */
class InputCheckValidScaled : public InputVisitor {
  rclcpp::Node::SharedPtr node_ = nullptr;
  std::shared_ptr<InputVisitor> next_ = nullptr;
  Logger logger_ = Logger("arm_teleop.input_check_valid_scaled");

 public:
  InputCheckValidScaled(const rclcpp::Node::SharedPtr& node,
                        const std::shared_ptr<InputVisitor>& next)
      : node_{node}, next_{next} {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }
  ~InputCheckValidScaled() override = default;
  void operator()(const geometry_msgs::msg::TwistStamped& command) override;
  void operator()(const control_msgs::msg::JointJog& command) override;
};

/**
 * @brief      Validates that the input messages do not contain NAN.
 */
class InputCheckValidSpeedUnits : public InputVisitor {
  rclcpp::Node::SharedPtr node_ = nullptr;
  std::shared_ptr<InputVisitor> next_ = nullptr;
  Logger logger_ = Logger("arm_teleop.input_check_valid_speed_units");

 public:
  InputCheckValidSpeedUnits(const rclcpp::Node::SharedPtr& node,
                            const std::shared_ptr<InputVisitor>& next)
      : node_{node}, next_{next} {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }
  ~InputCheckValidSpeedUnits() override = default;
  void operator()(const geometry_msgs::msg::TwistStamped& command) override;
  void operator()(const control_msgs::msg::JointJog& command) override;
};

/**
 * @brief      Makes an input check valid visitor.
 *
 * @param[in]  node             The node
 * @param[in]  command_in_type  The command in type.  Valid values are `scaled`
 * and `speed_units`.  Throws runtime_error if anyting else is passed in.
 * @param[in]  next             The next
 *
 * @return     A shared_ptr<InputVisitor>.
 */
std::shared_ptr<InputVisitor> makeInputCheckValid(
    const rclcpp::Node::SharedPtr& node, std::string_view command_in_type,
    const std::shared_ptr<InputVisitor>& next);

}  // namespace arm_teleop::detail
