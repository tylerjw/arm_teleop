
option(BUILD_TESTING "Enable Test Builds" ON)
if(BUILD_TESTING)
  # Gtest tests
  find_package(ament_cmake_gtest REQUIRED)

  # Input check valid
  ament_add_gtest(input_check_valid_tests input_check_valid_tests.cpp)
  target_link_libraries(input_check_valid_tests
    arm_teleop
    project_options
    ${control_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  )
endif()
