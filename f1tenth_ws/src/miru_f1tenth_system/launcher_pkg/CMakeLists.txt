cmake_minimum_required(VERSION 3.8)
project(launcher_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(odom_msgs REQUIRED)

add_executable(node_launcher src/node_launcher.cpp)
add_executable(js_node_launcher src/js_node_launcher.cpp)
add_executable(mk_node_launcher src/mk_node_launcher.cpp)
add_executable(sh_node_launcher src/sh_node_launcher.cpp)
add_executable(dg_node_launcher src/dg_node_launcher.cpp)
add_executable(hm_node_launcher src/hm_node_launcher.cpp)

target_include_directories(node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(js_node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mk_node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(sh_node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(dg_node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(hm_node_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)
ament_target_dependencies(js_node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)
ament_target_dependencies(mk_node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)
ament_target_dependencies(sh_node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)
ament_target_dependencies(dg_node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)
ament_target_dependencies(hm_node_launcher
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs odom_msgs
)

install(TARGETS node_launcher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS js_node_launcher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mk_node_launcher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sh_node_launcher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS dg_node_launcher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS hm_node_launcher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
