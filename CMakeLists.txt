cmake_minimum_required(VERSION 3.15)
project(arm_teleop)

include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
  if(ENABLE_BUILD_WITH_TIME_TRACE)
    target_compile_options(project_options INTERFACE -ftime-trace)
  endif()
endif()

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# enable cache system
include(cmake/Cache.cmake)

# Add linker configuration
include(cmake/Linker.cmake)
configure_linker(project_options)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# enable doxygen
include(cmake/Doxygen.cmake)
enable_doxygen()

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# find_package(moveit REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(control_toolbox REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(trajectory_msgs REQUIRED)

add_library(arm_teleop src/arm_teleop.cpp)
target_include_directories(arm_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  arm_teleop
  PRIVATE
    project_options
    project_warnings
    # moveit_core::moveit_robot_state
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(arm_teleop PRIVATE "ARM_TELEOP_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS arm_teleop
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(arm_teleop_node src/arm_teleop_node.cpp)
target_include_directories(arm_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  arm_teleop_node
  PRIVATE
    project_options
    project_warnings
    arm_teleop
)

install(TARGETS arm_teleop_node
  DESTINATION lib/${PROJECT_NAME})

option(BUILD_TESTING "Enable Test Builds" ON)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linters
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_flake8_FOUND TRUE) # Conflicts with black
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  arm_teleop
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
