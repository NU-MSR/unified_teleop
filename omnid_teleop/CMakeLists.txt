cmake_minimum_required(VERSION 3.14)
project(omnid_teleop)
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

set(OMNID_TELEOP_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs rosnu
)
set(OMNID_TELEOP_TARGET_LINK_LIBRARIES
  cmakeme_flags omnid_core::omnid_core yaml-cpp
  "${cpp_typesupport_target}"
)

# delta_joystick
add_executable(${PROJECT_NAME}_delta_joystick src/delta_joystick.cpp)
set_target_properties(${PROJECT_NAME}_delta_joystick
  PROPERTIES
  OUTPUT_NAME delta_joystick
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_delta_joystick
  ${OMNID_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_delta_joystick
  ${OMNID_TELEOP_TARGET_LINK_LIBRARIES}
)

# mobile_joystick
add_executable(${PROJECT_NAME}_mobile_joystick src/mobile_joystick.cpp)
set_target_properties(${PROJECT_NAME}_mobile_joystick
  PROPERTIES
  OUTPUT_NAME mobile_joystick
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_mobile_joystick
  ${OMNID_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_mobile_joystick
  ${OMNID_TELEOP_TARGET_LINK_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_delta_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_mobile_joystick
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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