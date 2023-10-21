cmake_minimum_required(VERSION 3.14)
project(unified_teleop)
find_package(cmakeme REQUIRED)
cmakeme_defaults(Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosnu REQUIRED)
find_package(omnid_core REQUIRED)
find_package(yaml-cpp REQUIRED)

###########
## Build ##
###########

ament_export_dependencies(rclcpp sensor_msgs nav_msgs rosnu)

set(UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs rosnu
)
set(UNIFIED_TELEOP_TARGET_LINK_LIBRARIES
  cmakeme_flags omnid_core::omnid_core yaml-cpp
  "${cpp_typesupport_target}"
)

# point_stamped_incr_joystick
add_executable(${PROJECT_NAME}_point_stamped_incr_joystick src/point_stamped_incr_joystick.cpp)
set_target_properties(${PROJECT_NAME}_point_stamped_incr_joystick
  PROPERTIES
  OUTPUT_NAME point_stamped_incr_joystick
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_point_stamped_incr_joystick
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_point_stamped_incr_joystick
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# point_stamped_mirror_joystick
add_executable(${PROJECT_NAME}_point_stamped_mirror_joystick src/point_stamped_mirror_joystick.cpp)
set_target_properties(${PROJECT_NAME}_point_stamped_mirror_joystick
  PROPERTIES
  OUTPUT_NAME point_stamped_mirror_joystick
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_point_stamped_mirror_joystick
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_point_stamped_mirror_joystick
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# cmd_vel_mirror_joystick
add_executable(${PROJECT_NAME}_cmd_vel_mirror_joystick src/cmd_vel_mirror_joystick.cpp)
set_target_properties(${PROJECT_NAME}_cmd_vel_mirror_joystick
  PROPERTIES
  OUTPUT_NAME cmd_vel_mirror_joystick
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_cmd_vel_mirror_joystick
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_cmd_vel_mirror_joystick
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# twist_merger
add_executable(${PROJECT_NAME}_twist_merger src/twist_merger.cpp)
set_target_properties(${PROJECT_NAME}_twist_merger
  PROPERTIES
  OUTPUT_NAME twist_merger
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_twist_merger
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_twist_merger
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_point_stamped_incr_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_point_stamped_mirror_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_cmd_vel_mirror_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_twist_merger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(catch_ros2 REQUIRED)

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

ament_package()