option(BUILD_TESTING "Enable Test Builds" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Input resampler
  ament_add_gtest(input_resampler_tests input_resampler_tests.cpp)
  target_link_libraries(input_resampler_tests
    arm_teleop
    project_options
    project_warnings
    ${control_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  )

  # Input stale command
  ament_add_gtest(input_stale_command_tests input_stale_command_tests.cpp)
  target_link_libraries(input_stale_command_tests
    arm_teleop
    project_options
    project_warnings
    ${control_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  )

  # Input subscriber
  ament_add_gtest(input_subscriber_tests input_subscriber_tests.cpp)
  target_link_libraries(input_subscriber_tests
    arm_teleop
    project_options
    project_warnings
    ${control_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  )

  # Input validate
  ament_add_gtest(input_validate_tests input_validate_tests.cpp)
  target_link_libraries(input_validate_tests
    arm_teleop
    project_options
    project_warnings
    ${control_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  )
endif()
