cmake_minimum_required(VERSION 3.5)
project(mikant_gazebo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(buoyancy_force_plugin SHARED src/buoyancy_force_plugin.cc)
add_library(lift_drag_force_plugin SHARED src/lift_drag_force_plugin.cc)
add_library(wind_world_plugin SHARED src/wind_world_plugin.cc)
add_library(heading_pid_controller_plugin SHARED src/heading_pid_controller_plugin.cc)



target_include_directories(buoyancy_force_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(buoyancy_force_plugin "gazebo_ros" "rclcpp")

target_include_directories(lift_drag_force_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(lift_drag_force_plugin "gazebo_ros" "rclcpp")

target_include_directories(wind_world_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(wind_world_plugin "gazebo_ros" "rclcpp")

target_include_directories(heading_pid_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(heading_pid_controller_plugin "gazebo_ros" "rclcpp")

install(TARGETS buoyancy_force_plugin DESTINATION share/${PROJECT_NAME})
install(TARGETS lift_drag_force_plugin DESTINATION share/${PROJECT_NAME})
install(TARGETS wind_world_plugin DESTINATION share/${PROJECT_NAME})
install(TARGETS heading_pid_controller_plugin DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
