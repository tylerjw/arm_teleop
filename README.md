# arm_teleop
[![Formatting (pre-commit)](https://github.com/tylerjw/arm_teleop/actions/workflows/format.yaml/badge.svg?branch=main)](https://github.com/tylerjw/arm_teleop/actions/workflows/format.yaml?query=branch%3Amain)
[![CI](https://github.com/tylerjw/arm_teleop/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/tylerjw/arm_teleop/actions/workflows/ci.yaml?query=branch%3Amain)
[![Pre-release](https://github.com/tylerjw/arm_teleop/actions/workflows/prerelease.yaml/badge.svg?branch=main)](https://github.com/tylerjw/arm_teleop/actions/workflows/prerelease.yaml?query=branch%3Amain)
[![Doxygen](https://github.com/tylerjw/arm_teleop/actions/workflows/doxygen.yaml/badge.svg?branch=main)](https://github.com/tylerjw/arm_teleop/actions/workflows/doxygen.yaml?query=branch%3Amain)
[![Code Coverage](https://codecov.io/gh/tylerjw/arm_teleop/branch/main/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/tylerjw/arm_teleop)

ROS 2 package for Teleoperation of a Robot Arm

[API Documentation](https://tylerjw.github.io/arm_teleop/)

## Building with sanitizers and linters

Here are the sanitizer and linter builds done in CI:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
                          -DENABLE_SANITIZER_ADDRESS=ON \
                          -DENABLE_SANITIZER_LEAK=ON \
                          -DENABLE_SANITIZER_UNDEFINED_BEHAVIOR=ON
```

```bash
CXX=clang++ \
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
                          -DENABLE_SANITIZER_THREAD=ON
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
                          -DENABLE_SANITIZER_MEMORY=ON
```

```bash
colcon build --cmake-args -DENABLE_INCLUDE_WHAT_YOU_USE=ON \
                          -DENABLE_CLANG_TIDY=ON \
             --event-handlers console_direct+
```

Run tests with console output:
```bash
colcon test --event-handlers console_direct+
```

## Building doxygen site

Build:
```bash
colcon build --cmake-args -DENABLE_DOXYGEN=ON \
             --cmake-target doxygen-docs \
             --event-handlers console_direct+
```

Open in browser:
```bash
open build/arm_teleop/html/index.html
```

### Achnologements

- Inspired by [moveit_servo](https://github.com/ros-planning/moveit2/tree/main/moveit_ros/moveit_servo)
- CMake copied from [cpp-best-practices/cpp_starter_project](https://github.com/cpp-best-practices/cpp_starter_project)
