cmake_minimum_required(VERSION 3.5)
project(origincar_task)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# include
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# task_master executable
add_executable(task_master_node
  src/task_master.cpp
)

ament_target_dependencies(task_master_node
  rclcpp
  ai_msgs
  ament_index_cpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  yaml-cpp
)

# Link non-ament libraries
target_link_libraries(task_master_node
  yaml-cpp
)

# install
install(TARGETS task_master_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/generate_map_semantics.py
  scripts/imu_calibrate.py
  scripts/odom_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_generate_map_semantics test/test_generate_map_semantics.py)
  ament_add_pytest_test(test_competition_config test/test_competition_config.py)
  ament_add_gtest(test_mission_policy test/test_mission_policy.cpp)
  target_include_directories(test_mission_policy PRIVATE include)
endif()

ament_package()
