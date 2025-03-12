cmake_minimum_required(VERSION 3.5)
project(odom_publisher)

# Setting C++ standard to 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(odom_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/vesc_to_odom_with_ekf.cpp
)

# Specify dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  ackermann_msgs
  vesc_msgs
  odom_msgs
)

# Link Eigen
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# Register component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::VescToOdomWithEKF
  EXECUTABLE vesc_to_odom_with_ekf_node
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
