cmake_minimum_required(VERSION 3.5)
project(odom_navigation)

# C++ 표준 설정 (예: C++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(nav_msgs REQUIRED)
#find_package(vesc_msgs REQUIRED)
find_package(odom_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 실행 파일 생성 (소스 파일은 src/odom_node.cpp)
add_executable(odom_navigation_node src/odom_navigation_node.cpp)
ament_target_dependencies(odom_navigation_node 
  rclcpp 
 # vesc_msgs
  odom_msgs 
  ackermann_msgs 
  tf2 
  tf2_geometry_msgs
)

# 설치 설정: 실행 파일을 lib/<패키지명>에 설치
install(TARGETS
  odom_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
