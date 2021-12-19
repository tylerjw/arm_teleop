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
#include <atomic>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <variant>

using arm_teleop::detail::InputCommand;
using arm_teleop::detail::InputVisitor;
using control_msgs::msg::JointJog;
using geometry_msgs::msg::TwistStamped;

class DoNothingVisitor : public InputVisitor {
 public:
  void operator()(const TwistStamped& /*unused*/) override{};
  void operator()(const JointJog& /*unused*/) override{};
  ~DoNothingVisitor() override = default;
};

class CountingVisitor : public InputVisitor {
 public:
  std::atomic<unsigned int> count = 0;
  void operator()(const TwistStamped& /*unused*/) override { count++; };
  void operator()(const JointJog& /*unused*/) override { count++; };
  ~CountingVisitor() override = default;
};

class TypeCountingVisitor : public InputVisitor {
 public:
  std::atomic<unsigned int> twist_stamped_count = 0;
  std::atomic<unsigned int> joint_jog_count = 0;
  void operator()(const TwistStamped& /*unused*/) override {
    twist_stamped_count++;
  };
  void operator()(const JointJog& /*unused*/) override { joint_jog_count++; };
  ~TypeCountingVisitor() override = default;
};

class ReceivedCommandVisitor : public InputVisitor {
  mutable std::mutex mutex_;
  InputCommand received_command_;

 public:
  void operator()(const TwistStamped& command) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    received_command_ = command;
  };
  void operator()(const JointJog& command) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    received_command_ = command;
  };
  ~ReceivedCommandVisitor() override = default;

  InputCommand getReceivedCommand() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return received_command_;
  }
};
