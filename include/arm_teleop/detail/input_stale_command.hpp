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
#include <variant>

namespace arm_teleop::detail {
/**
 * @brief      Visitor that sets the header.stamp to now and then calls the next
 * visitor
 */
class TimestampNow : public InputVisitor {
  rclcpp::Node::SharedPtr node_ = nullptr;
  std::shared_ptr<InputVisitor> next_ = nullptr;

 public:
  TimestampNow(const rclcpp::Node::SharedPtr& node,
               const std::shared_ptr<InputVisitor>& next)
      : node_{node}, next_{next} {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }
  ~TimestampNow() override = default;

  void operator()(const geometry_msgs::msg::TwistStamped& command) override {
    operatorImpl(command);
  }

  void operator()(const control_msgs::msg::JointJog& command) override {
    operatorImpl(command);
  }

 private:
  // Note that this implementation template takes the input command by value
  // because it mutates it before visiting the next step.
  template <typename T>
  void operatorImpl(T command) {
    command.header.stamp = node_->now();
    std::visit(*next_, InputCommand{command});
  }
};

/**
 * @brief      Visitor that if the message is stale it calls halt, otherwise it
 * calls the next visitor
 */
class StaleCommandHalt : public InputVisitor {
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Duration timeout_;
  std::function<void()> halt_;
  std::shared_ptr<InputVisitor> next_ = nullptr;

 public:
  StaleCommandHalt(const rclcpp::Node::SharedPtr& node,
                   const rclcpp::Duration& timeout, std::function<void()> halt,
                   const std::shared_ptr<InputVisitor>& next)
      : node_{node}, timeout_{timeout}, halt_{halt}, next_{next} {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }
  ~StaleCommandHalt() override = default;

  void operator()(const geometry_msgs::msg::TwistStamped& command) override {
    operatorImpl(command);
  }

  void operator()(const control_msgs::msg::JointJog& command) override {
    operatorImpl(command);
  }

 private:
  template <typename T>
  void operatorImpl(const T& command) {
    if (node_->now() - command.header.stamp > timeout_) {
      halt_();
    } else {
      std::visit(*next_, InputCommand{command});
    }
  }
};

}  // namespace arm_teleop::detail
