cmake_minimum_required(VERSION 3.8)
project(roboclaw_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # TODO remove
find_package(roboclaw-cpp REQUIRED)

# Build

# roboclaw_utils library
file(GLOB_RECURSE ROBOCLAW_UTILS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/utils/*)
add_library(roboclaw_utils ${ROBOCLAW_UTILS_SRC})
target_include_directories(roboclaw_utils
  PUBLIC include
)
target_link_libraries(roboclaw_utils
  roboclaw::roboclaw-cpp
)
ament_target_dependencies(roboclaw_utils
  rclcpp
)

# roboclaw_controller node
add_executable(roboclaw_controller src/nodes/roboclaw_controller.cpp)
target_link_libraries(roboclaw_controller
  roboclaw_utils
)
ament_target_dependencies(roboclaw_controller
  rclcpp
  std_msgs # TODO remove
)

# Install

# Nodes
install(TARGETS
  roboclaw_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
