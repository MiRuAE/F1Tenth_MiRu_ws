cmake_minimum_required(VERSION 3.5)
project(odom_publisher)

# Setting C++ standard to 14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(odom_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# nodes library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vesc_to_odom_with_ekf.cpp
)

# 이제 타겟이 생성되었으므로 include 디렉토리를 지정합니다.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# register nodes as components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_ackermann::VescToOdomWithEKF
  EXECUTABLE vesc_to_odom_with_ekf_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_auto_package(
#  INSTALL_TO_SHARE
#    launch
#)
