cmake_minimum_required(VERSION 3.8)
project(bot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

install(
  DIRECTORY
    launch
    behavior_trees
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/costmap_center_check.py
  scripts/nav2_utils.py
  scripts/nav_odom_relay.py
  scripts/navigation_preflight.py
  scripts/waypoint_recorder.py
  scripts/waypoint_navigator.py
  scripts/localization_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Nav2's bt_navigator handles the namespaced goal_pose topic directly. Remove
# the obsolete bridge from existing install spaces so it cannot be restarted.
install(CODE
  "file(REMOVE \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/goal_pose_bridge.py\")"
)
ament_python_install_package(${PROJECT_NAME})

ament_package()
