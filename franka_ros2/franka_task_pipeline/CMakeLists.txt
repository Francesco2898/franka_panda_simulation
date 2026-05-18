cmake_minimum_required(VERSION 3.8)
project(franka_task_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

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

add_executable(keyboard_cmd_node src/keyboard_cmd_node.cpp)
ament_target_dependencies(keyboard_cmd_node rclcpp std_msgs)
install(TARGETS keyboard_cmd_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(task_pipeline_node src/task_pipeline_node.cpp)
ament_target_dependencies(task_pipeline_node rclcpp std_msgs)
install(TARGETS task_pipeline_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
