cmake_minimum_required(VERSION 3.14)
project(unified_teleop)
find_package(cmakeme REQUIRED)
cmakeme_defaults(Debug)

# find dependencies
find_package(catch_ros2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

###########
## Build ##
###########

# Create a library
add_library(unified_teleop src/controller.cpp include/unified_teleop/param_helpers.hpp src/pnt_stmp_modifiers.cpp src/twist_modifiers.cpp)

# Include the include directories and their header files
target_include_directories(unified_teleop
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(unified_teleop
  rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle
)
target_link_libraries(unified_teleop
  cmakeme_flags yaml-cpp "${cpp_typesupport_target}"
)

# defining common dependencies
set(COMMON_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  rosbag2_interfaces
  lifecycle_msgs
  rclcpp_lifecycle
)

# defining common libraries
set(COMMON_LIBS
  unified_teleop
  cmakeme_flags
  yaml-cpp
  "${cpp_typesupport_target}"
)

# point_stamped_incr
add_executable(point_stamped_incr_node src/point_stamped_incr.cpp)
set_target_properties(point_stamped_incr_node
  PROPERTIES
  OUTPUT_NAME point_stamped_incr
  PREFIX ""
)
ament_target_dependencies(point_stamped_incr_node ${COMMON_DEPS})
target_link_libraries(point_stamped_incr_node
  ${COMMON_LIBS}
)

# point_stamped_mirror
add_executable(point_stamped_mirror_node src/point_stamped_mirror.cpp)
set_target_properties(point_stamped_mirror_node
  PROPERTIES
  OUTPUT_NAME point_stamped_mirror
  PREFIX ""
)
ament_target_dependencies(point_stamped_mirror_node ${COMMON_DEPS})
target_link_libraries(point_stamped_mirror_node
  ${COMMON_LIBS}
)

# twist_mirror
add_executable(teleop_twist_mirror_node src/twist_mirror.cpp)
set_target_properties(teleop_twist_mirror_node
  PROPERTIES
  OUTPUT_NAME twist_mirror
  PREFIX ""
)
ament_target_dependencies(teleop_twist_mirror_node ${COMMON_DEPS})
target_link_libraries(teleop_twist_mirror_node
  ${COMMON_LIBS}
)

# twist_merger
add_executable(teleop_twist_merger_node src/twist_merger.cpp)
set_target_properties(teleop_twist_merger_node
  PROPERTIES
  OUTPUT_NAME twist_merger
  PREFIX ""
)
ament_target_dependencies(teleop_twist_merger_node ${COMMON_DEPS})
target_link_libraries(teleop_twist_merger_node
  ${COMMON_LIBS}
)

# lifecycle_point_stamped_incr
add_executable(lifecycle_point_stamped_incr_node src/lifecycle_point_stamped_incr.cpp)
set_target_properties(lifecycle_point_stamped_incr_node
  PROPERTIES
  OUTPUT_NAME lifecycle_point_stamped_incr
  PREFIX ""
)
ament_target_dependencies(lifecycle_point_stamped_incr_node ${COMMON_DEPS})
target_link_libraries(lifecycle_point_stamped_incr_node
  ${COMMON_LIBS}
)

# lifecycle_twist_mirror
add_executable(lifecycle_twist_mirror_node src/lifecycle_twist_mirror.cpp)
set_target_properties(lifecycle_twist_mirror_node
  PROPERTIES
  OUTPUT_NAME lifecycle_twist_mirror
  PREFIX ""
)
ament_target_dependencies(lifecycle_twist_mirror_node ${COMMON_DEPS})
target_link_libraries(lifecycle_twist_mirror_node
  ${COMMON_LIBS}
)

# # integration_test
# add_executable(unified_teleop_integration_test_node test/integration_test_node.cpp)
# set_target_properties(unified_teleop_integration_test_node
#   PROPERTIES
#   OUTPUT_NAME integration_test_node
#   PREFIX ""
# )
# ament_target_dependencies(unified_teleop_integration_test_node
#   rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle
# )
# target_link_libraries(unified_teleop_integration_test_node
#  cmakeme_flags yaml-cpp "${cpp_typesupport_target}"
#   catch_ros2::catch_ros2_with_node_main
# )
#
# target_link_libraries(integration_test_node
#   catch_ros2::catch_ros2_with_node_main
# )

#############
## Install ##
#############

install(TARGETS
  point_stamped_incr_node
  RUNTIME DESTINATION lib/unified_teleop
)

install(TARGETS
  point_stamped_mirror_node
  RUNTIME DESTINATION lib/unified_teleop
)

install(TARGETS
  teleop_twist_mirror_node
  RUNTIME DESTINATION lib/unified_teleop
)

install(TARGETS
  teleop_twist_merger_node
  RUNTIME DESTINATION lib/unified_teleop
)

install(TARGETS
  lifecycle_point_stamped_incr_node
  RUNTIME DESTINATION lib/unified_teleop
)

install(TARGETS
  lifecycle_twist_mirror_node
  RUNTIME DESTINATION lib/unified_teleop
)

# install example files
install(DIRECTORY example DESTINATION share/unified_teleop)

# install configuration files
install(DIRECTORY config DESTINATION share/unified_teleop)

# install testing files
install(DIRECTORY test DESTINATION share/unified_teleop)

#############
## Testing ##
#############

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

  ###########################
  ## INTEGRATION TEST NODE ##
  ###########################
  include(CTest)

  # integration_test
  add_executable(unified_teleop_integration_test_node test/integration_test_node.cpp)
  set_target_properties(unified_teleop_integration_test_node
    PROPERTIES
    OUTPUT_NAME integration_test_node
    PREFIX ""
  )
  ament_target_dependencies(unified_teleop_integration_test_node
    rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle
  )
  target_link_libraries(unified_teleop_integration_test_node
    cmakeme_flags yaml-cpp "${cpp_typesupport_target}"
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    unified_teleop_integration_test_node
    RUNTIME DESTINATION lib/unified_teleop
  )

  # playground_test
  add_executable(unified_teleop_playground_test_node test/playground_test_node.cpp)
  set_target_properties(unified_teleop_playground_test_node
    PROPERTIES
    OUTPUT_NAME playground_test_node
    PREFIX ""
  )
  ament_target_dependencies(unified_teleop_playground_test_node
    rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle
  )
  target_link_libraries(unified_teleop_playground_test_node
    cmakeme_flags yaml-cpp "${cpp_typesupport_target}"
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    unified_teleop_playground_test_node
    RUNTIME DESTINATION lib/unified_teleop
  )

  #################
  ## LAUNCH FILE ##
  #################
  # # This is the launch file that will be used to run the integration test
  # install(FILES
  #   test/integration_test.launch.xml  # example in XML
  #   DESTINATION share/unified_teleop
  # )

  endif()

ament_package()