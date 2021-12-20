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

#include <arm_teleop/detail/input_command.hpp>
#include <arm_teleop/detail/input_stale_command.hpp>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <stdexcept>
#include <variant>

#include "gtest/gtest-message.h"
#include "gtest/gtest-test-part.h"
#include "gtest/gtest_pred_impl.h"
#include "input_visitors.hpp"

namespace {
const auto TIME_EPSILON = 1e-1;
}  // namespace

using arm_teleop::detail::InputCommand;
using arm_teleop::detail::StaleCommandHalt;
using arm_teleop::detail::TimestampNow;

TEST(InputReamplerTests, NullNodeTimestampNow)  // NOLINT
{
  // GIVEN a valid visitor
  // WHEN we construct TimestampNow with nullptr for node
  // THEN we expect it to throw
  auto visitor = std::make_shared<CountingVisitor>();
  EXPECT_THROW(  // NOLINT
      auto resampler = TimestampNow(nullptr, visitor), std::runtime_error);
}

TEST(InputReamplerTests, NullVisitorTimestampNow)  // NOLINT
{
  // GIVEN a valid node
  // WHEN we construct TimestampNow with nullptr for visitor
  // THEN we expect it to throw
  auto node = std::make_shared<rclcpp::Node>("_");
  EXPECT_THROW(  // NOLINT
      auto resampler = TimestampNow(node, nullptr), std::runtime_error);
}

TEST(InputStaleCommandTests, TimeStampIsNearNowJointJog)  // NOLINT
{
  // GIVEN a TimestampNow with with a Visitor that saves modified message
  // locally
  auto visitor = std::make_shared<ReceivedCommandVisitor>();
  auto node = std::make_shared<rclcpp::Node>("_");
  auto timestamp_now = TimestampNow(node, visitor);

  // WHEN we visit the timestamp_now with one message
  auto sent_msg = JointJog{};
  std::visit(timestamp_now, InputCommand{sent_msg});

  // THEN we expect the stamp to have changed
  ASSERT_TRUE(
      std::holds_alternative<decltype(sent_msg)>(visitor->getReceivedCommand()))
      << "Received command should be of the same type as sent message";
  auto received_msg = std::get<JointJog>(visitor->getReceivedCommand());
  EXPECT_NE(received_msg.header.stamp, sent_msg.header.stamp)
      << "Received message should have a different timestamp than sent one";
  EXPECT_NEAR((node->now() - received_msg.header.stamp).seconds(), 0,
              TIME_EPSILON)
      << "Time difference between now and received_msg should be close to 0";
}

TEST(InputStaleCommandTests, TimeStampIsNearNowTwistStamped)  // NOLINT
{
  // GIVEN a TimestampNow with with a Visitor that saves modified message
  // locally
  auto visitor = std::make_shared<ReceivedCommandVisitor>();
  auto node = std::make_shared<rclcpp::Node>("_");
  auto timestamp_now = TimestampNow(node, visitor);

  // WHEN we visit the timestamp_now with one message
  auto sent_msg = TwistStamped{};
  std::visit(timestamp_now, InputCommand{sent_msg});

  // THEN we expect the stamp to have changed
  ASSERT_TRUE(
      std::holds_alternative<decltype(sent_msg)>(visitor->getReceivedCommand()))
      << "Received command should be of the same type as sent message";
  auto received_msg = std::get<TwistStamped>(visitor->getReceivedCommand());
  EXPECT_NE(received_msg.header.stamp, sent_msg.header.stamp)
      << "Received message should have a different timestamp than sent one";
  EXPECT_NEAR((node->now() - received_msg.header.stamp).seconds(), 0,
              TIME_EPSILON)
      << "Time difference between now and received_msg should be close to 0";
}

TEST(InputReamplerTests, NullNodeStaleCommandHalt)  // NOLINT
{
  // GIVEN a valid visitor
  // WHEN we construct StaleCommandHalt with nullptr for node
  // THEN we expect it to throw
  auto visitor = std::make_shared<CountingVisitor>();
  EXPECT_THROW(  // NOLINT
      auto resampler = StaleCommandHalt(
          nullptr, rclcpp::Duration::from_seconds(1), [&]() {}, visitor),
      std::runtime_error);
}

TEST(InputReamplerTests, NullVisitorStaleCommandHalt)  // NOLINT
{
  // GIVEN a valid node
  // WHEN we construct StaleCommandHalt with nullptr for visitor
  // THEN we expect it to throw
  auto node = std::make_shared<rclcpp::Node>("_");
  EXPECT_THROW(  // NOLINT
      auto resampler = StaleCommandHalt(
          node, rclcpp::Duration::from_seconds(1), [&]() {}, nullptr),
      std::runtime_error);
}

TEST(InputStaleCommandTests, HaltTwistStamped)  // NOLINT
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts
  // calls
  auto visitor = std::make_shared<CountingVisitor>();
  unsigned int halt_count{0};
  auto stale_command_halt = StaleCommandHalt(
      std::make_shared<rclcpp::Node>("_"), rclcpp::Duration::from_seconds(1),
      [&]() { halt_count++; }, visitor);

  // WHEN we visit the stale_command_halt with one message that is default
  // constructed
  std::visit(stale_command_halt, InputCommand{TwistStamped{}});

  // THEN we expect the visitor count to still be 0 and the halt count should be
  // 1
  EXPECT_EQ(visitor->count, 0U) << "Visitor should not have been visited";
  EXPECT_EQ(halt_count, 1U) << "Halt should have been called once";
}

TEST(InputStaleCommandTests, HaltJointJog)  // NOLINT
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts
  // calls
  auto visitor = std::make_shared<CountingVisitor>();
  unsigned int halt_count{0};
  auto stale_command_halt = StaleCommandHalt(
      std::make_shared<rclcpp::Node>("_"), rclcpp::Duration::from_seconds(1),
      [&]() { halt_count++; }, visitor);

  // WHEN we visit the stale_command_halt with one message that is default
  // constructed
  std::visit(stale_command_halt, InputCommand{JointJog{}});

  // THEN we expect the visitor count to still be 0 and the halt count should be
  // 1
  EXPECT_EQ(visitor->count, 0U) << "Visitor should not have been visited";
  EXPECT_EQ(halt_count, 1U) << "Halt should have been called once";
}

TEST(InputStaleCommandTests, NotStaleVisitTwistStamped)  // NOLINT
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts
  // calls
  auto visitor = std::make_shared<CountingVisitor>();
  unsigned int halt_count{0};
  auto node = std::make_shared<rclcpp::Node>("_");
  auto stale_command_halt = StaleCommandHalt(
      node, rclcpp::Duration::from_seconds(1), [&]() { halt_count++; },
      visitor);

  // WHEN we visit the stale_command_halt with one message that has a current
  // timestamp
  auto sent_msg = TwistStamped{};
  sent_msg.header.stamp = node->now();
  std::visit(stale_command_halt, InputCommand{sent_msg});

  // THEN we expect the visitor count to be 1 and the halt count should be 0
  EXPECT_EQ(visitor->count, 1U) << "Visitor should have been visited";
  EXPECT_EQ(halt_count, 0U) << "Halt should not have been called once";
}

TEST(InputStaleCommandTests, NotStaleVisitJointJog)  // NOLINT
{
  // GIVEN a StaleCommandHalt with with a Visitor and halt function that counts
  // calls
  auto visitor = std::make_shared<CountingVisitor>();
  unsigned int halt_count{0};
  auto node = std::make_shared<rclcpp::Node>("_");
  auto stale_command_halt = StaleCommandHalt(
      node, rclcpp::Duration::from_seconds(1), [&]() { halt_count++; },
      visitor);

  // WHEN we visit the stale_command_halt with one message that has a current
  // timestamp
  auto sent_msg = JointJog{};
  sent_msg.header.stamp = node->now();
  std::visit(stale_command_halt, InputCommand{sent_msg});

  // THEN we expect the visitor count to be 1 and the halt count should be 0
  EXPECT_EQ(visitor->count, 1U) << "Visitor should have been visited";
  EXPECT_EQ(halt_count, 0U) << "Halt should not have been called once";
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
