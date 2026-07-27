cmake_minimum_required(VERSION 3.8)
project(g1_loop_closure)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(loop_closure_node
  src/loop_closure_node.cpp
  src/pose_graph.cpp
)
target_include_directories(loop_closure_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(loop_closure_node Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_options(loop_closure_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(loop_closure_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

install(TARGETS loop_closure_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
