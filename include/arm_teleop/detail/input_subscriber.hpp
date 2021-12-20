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
#include <control_msgs/msg/joint_jog.hpp>
#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <variant>

namespace arm_teleop::detail {
/**
 * @brief      Subscribes to the two input topics and calls visitor on them
 */
class InputSubscriber {
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_stamped_sub_ = nullptr;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_cmd_sub_ =
      nullptr;

 public:
  InputSubscriber(const rclcpp::Node::SharedPtr& node,
                  const std::string& twist_stamped_topic,
                  const std::string& joint_jog_topic,
                  std::shared_ptr<InputVisitor> next) {
    if (!static_cast<bool>(node)) {
      throw std::runtime_error("Node was not valid");
    }
    if (!static_cast<bool>(next)) {
      throw std::runtime_error("Next Visitor was not valid");
    }
    twist_stamped_sub_ =
        node->create_subscription<geometry_msgs::msg::TwistStamped>(
            twist_stamped_topic, rclcpp::SystemDefaultsQoS(),
            [=](std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
              std::visit(*next, InputCommand{*msg});
            });
    joint_cmd_sub_ = node->create_subscription<control_msgs::msg::JointJog>(
        joint_jog_topic, rclcpp::SystemDefaultsQoS(),
        [=](std::shared_ptr<control_msgs::msg::JointJog> msg) {
          std::visit(*next, InputCommand{*msg});
        });
  }
};

}  // namespace arm_teleop::detail
