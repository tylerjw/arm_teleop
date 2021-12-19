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
#include <cmath>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string_view>
#include <variant>
#include <vector>

#include "arm_teleop/detail/input_check_valid.hpp"
#include "gtest/gtest-message.h"
#include "gtest/gtest-test-part.h"
#include "gtest/gtest_pred_impl.h"
#include "input_visitors.hpp"

using arm_teleop::detail::InputCheckValid;
using arm_teleop::detail::InputCommand;

TEST(InputCheckValidTests, DefaultTwistStamp)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "unitless", visitor);

  // WHEN we visit with a default constructed TwistStamped
  std::visit(input_check_valid, InputCommand{TwistStamped{}});

  // THEN we expect the visitor to have been called
  ASSERT_EQ(visitor->count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, DefaultJointJog)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "unitless", visitor);

  // WHEN we visit with a default constructed JointJog
  std::visit(input_check_valid, InputCommand{JointJog{}});

  // THEN we expect the visitor to have been called
  ASSERT_EQ(visitor->count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, InvalidUnitlessTwistStamp)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "unitless", visitor);

  // WHEN we visit with a TwistStamped that is invlid in "unitless mode"
  auto command = TwistStamped{};
  const auto kVelocity = 10.0;
  command.twist.angular.z = kVelocity;
  std::visit(input_check_valid, InputCommand{command});

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor->count, 0U) << "The visitor should NOT have been called.";
}

TEST(InputCheckValidTests, ValidNotUnitlessTwistStamp)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "", visitor);

  // WHEN we visit with a TwistStamped that is invlid in "unitless mode"
  auto command = TwistStamped{};
  const auto kVelocity = 10.0;
  command.twist.angular.z = kVelocity;
  std::visit(input_check_valid, InputCommand{command});

  // THEN we expect the visitor was called
  ASSERT_EQ(visitor->count, 1U) << "The visitor should have been called.";
}

TEST(InputCheckValidTests, NanTwistStamp)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "unitless", visitor);

  // WHEN we visit with a TwistStamped containing a NaN
  auto command = TwistStamped{};
  command.twist.angular.z = NAN;
  std::visit(input_check_valid, InputCommand{command});

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor->count, 0U) << "The visitor should NOT have been called.";
}

TEST(InputCheckValidTests, NanJointJog)  // NOLINT
{
  // GIVEN a InputCheckValid with with a Visitor that counts
  auto visitor = std::make_shared<CountingVisitor>();
  auto input_check_valid =
      InputCheckValid(std::make_shared<rclcpp::Node>("_"), "unitless", visitor);

  // WHEN we visit with a JointJog containing a NaN
  auto command = JointJog{};
  command.velocities.push_back(NAN);
  std::visit(input_check_valid, InputCommand{command});

  // THEN we expect the visitor was not called
  ASSERT_EQ(visitor->count, 0U) << "The visitor should NOT have been called.";
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
