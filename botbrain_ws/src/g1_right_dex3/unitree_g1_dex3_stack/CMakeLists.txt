cmake_minimum_required(VERSION 3.5)
project(unitree_g1_dex3_stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_IK_FCL_OMPL_PLANNER "Build the IK/FCL/OMPL planner node" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(BUILD_IK_FCL_OMPL_PLANNER)
  find_package(trac_ik_lib REQUIRED)
  find_package(ompl REQUIRED)
  find_package(fcl CONFIG REQUIRED)
  find_package(geometric_shapes REQUIRED)
  find_package(resource_retriever REQUIRED)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

if(BUILD_IK_FCL_OMPL_PLANNER)
  include_directories(
    ${OMPL_INCLUDE_DIRS}
    ${TRAC_IK_INCLUDE_DIRS}
  )
endif()

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
add_executable(dex3_controller src/dex3_controller.cpp)
add_executable(joint_trajectory_executor src/joint_trajectory_executor.cpp)
add_executable(right_hand_pressure_monitor src/right_hand_pressure_monitor.cpp)
add_executable(visual_detection_tester src/visual_detection_tester.cpp)

if(BUILD_IK_FCL_OMPL_PLANNER)
  add_executable(ik_fcl_ompl_planner src/ik_fcl_ompl_planner.cpp)
endif()

ament_target_dependencies(joint_state_publisher
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs urdf kdl_parser
)
ament_target_dependencies(dex3_controller
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs urdf
)
ament_target_dependencies(joint_trajectory_executor
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs trajectory_msgs urdf kdl_parser
)
ament_target_dependencies(right_hand_pressure_monitor
  rclcpp unitree_hg
)
ament_target_dependencies(visual_detection_tester
  rclcpp sensor_msgs geometry_msgs cv_bridge message_filters tf2 tf2_ros tf2_geometry_msgs
)
if(BUILD_IK_FCL_OMPL_PLANNER)
  ament_target_dependencies(ik_fcl_ompl_planner
    rclcpp sensor_msgs geometry_msgs std_msgs trajectory_msgs urdf kdl_parser trac_ik_lib geometric_shapes resource_retriever vision_msgs ament_index_cpp tf2 tf2_ros tf2_geometry_msgs
  )

  target_link_libraries(ik_fcl_ompl_planner
    ${OMPL_LIBRARIES}
    fcl
    ${TRAC_IK_LIBRARIES}
  )
endif()

target_link_libraries(visual_detection_tester
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  joint_state_publisher
  dex3_controller
  joint_trajectory_executor
  right_hand_pressure_monitor
  visual_detection_tester
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_IK_FCL_OMPL_PLANNER)
  install(TARGETS
    ik_fcl_ompl_planner
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/tcp_torso_pose.py
  scripts/apriltag_detector_node.py
  scripts/v4l2_apriltag_trigger.py
  scripts/adaptive_orientation_ab.py
  scripts/apriltag_goal_bridge.py
  scripts/apriltag_button_press_node.py
  scripts/apriltag_reach_uat.py
  scripts/gravity_torque_publisher.py
  scripts/button_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  robots
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
