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

# ament_export_dependencies(rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle)

set(UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES
  rclcpp geometry_msgs sensor_msgs rosbag2_interfaces lifecycle_msgs rclcpp_lifecycle
)

# Create a library
add_library(${PROJECT_NAME} src/controller.cpp include/unified_teleop/param_helpers.hpp include/unified_teleop/pnt_stmp_modifiers.hpp)

# Include the include directories and their header files
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(${PROJECT_NAME}
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

set(UNIFIED_TELEOP_TARGET_LINK_LIBRARIES
  cmakeme_flags yaml-cpp ${PROJECT_NAME}
  "${cpp_typesupport_target}"
)

# point_stamped_incr
add_executable(${PROJECT_NAME}_point_stamped_incr src/point_stamped_incr.cpp)
set_target_properties(${PROJECT_NAME}_point_stamped_incr
  PROPERTIES
  OUTPUT_NAME point_stamped_incr
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_point_stamped_incr
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_point_stamped_incr
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# point_stamped_mirror
add_executable(${PROJECT_NAME}_point_stamped_mirror src/point_stamped_mirror.cpp)
set_target_properties(${PROJECT_NAME}_point_stamped_mirror
  PROPERTIES
  OUTPUT_NAME point_stamped_mirror
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_point_stamped_mirror
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_point_stamped_mirror
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# twist_mirror
add_executable(${PROJECT_NAME}_twist_mirror src/twist_mirror.cpp)
set_target_properties(${PROJECT_NAME}_twist_mirror
  PROPERTIES
  OUTPUT_NAME twist_mirror
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_twist_mirror
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_twist_mirror
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

# lifecycle_point_stamped_incr
add_executable(${PROJECT_NAME}_lifecycle_point_stamped_incr src/lifecycle_point_stamped_incr.cpp)
set_target_properties(${PROJECT_NAME}_lifecycle_point_stamped_incr
  PROPERTIES
  OUTPUT_NAME lifecycle_point_stamped_incr
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_lifecycle_point_stamped_incr
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_lifecycle_point_stamped_incr
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# lifecycle_twist_mirror
add_executable(${PROJECT_NAME}_lifecycle_twist_mirror src/lifecycle_twist_mirror.cpp)
set_target_properties(${PROJECT_NAME}_lifecycle_twist_mirror
  PROPERTIES
  OUTPUT_NAME lifecycle_twist_mirror
  PREFIX ""
)
ament_target_dependencies(${PROJECT_NAME}_lifecycle_twist_mirror
  ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}_lifecycle_twist_mirror
  ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
)

# # integration_test
# add_executable(${PROJECT_NAME}_integration_test_node test/integration_test_node.cpp)
# set_target_properties(${PROJECT_NAME}_integration_test_node
#   PROPERTIES
#   OUTPUT_NAME integration_test_node
#   PREFIX ""
# )
# ament_target_dependencies(${PROJECT_NAME}_integration_test_node
#   ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
# )
# target_link_libraries(${PROJECT_NAME}_integration_test_node
#   ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
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
  ${PROJECT_NAME}_point_stamped_incr
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_point_stamped_mirror
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_twist_mirror
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_twist_merger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lifecycle_point_stamped_incr
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lifecycle_twist_mirror
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install example files
install(DIRECTORY example DESTINATION share/${PROJECT_NAME})

# install configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# install testing files
install(DIRECTORY test DESTINATION share/${PROJECT_NAME})

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
  add_executable(${PROJECT_NAME}_integration_test_node test/integration_test_node.cpp)
  set_target_properties(${PROJECT_NAME}_integration_test_node
    PROPERTIES
    OUTPUT_NAME integration_test_node
    PREFIX ""
  )
  ament_target_dependencies(${PROJECT_NAME}_integration_test_node
    ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_integration_test_node
    ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    ${PROJECT_NAME}_integration_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  # playground_test
  add_executable(${PROJECT_NAME}_playground_test_node test/playground_test_node.cpp)
  set_target_properties(${PROJECT_NAME}_playground_test_node
    PROPERTIES
    OUTPUT_NAME playground_test_node
    PREFIX ""
  )
  ament_target_dependencies(${PROJECT_NAME}_playground_test_node
    ${UNIFIED_TELEOP_AMENT_TARGET_DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_playground_test_node
    ${UNIFIED_TELEOP_TARGET_LINK_LIBRARIES}
    catch_ros2::catch_ros2_with_node_main
  )
  install(TARGETS
    ${PROJECT_NAME}_playground_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  #################
  ## LAUNCH FILE ##
  #################
  # # This is the launch file that will be used to run the integration test
  # install(FILES
  #   test/integration_test.launch.xml  # example in XML
  #   DESTINATION share/${PROJECT_NAME}
  # )

  endif()

ament_package()