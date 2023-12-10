// Copyright 2023 Nick Morales.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file This is an example ROS 2 node that checks assertions using Catch2.
/// It simply checks if the "test_service" service is available at least once
/// during the duration of the test.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// Saved messages from the different Teleop Nodes
geometry_msgs::msg::PointStamped pntstmp_mir_msg;

/// @brief Returns a Joy message that with rounded values to a certain decimal point
/// @param input_command - The PointStamped message that will be rounded
static void pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state);

/// @brief Returns a Joy message that with rounded values to a certain decimal point
/// @param input_command - The PointStamped message that will be rounded
static sensor_msgs::msg::Joy joy_msg(const std::vector<float>& axis_array, const std::vector<int>& button_array);

TEST_CASE("integration_test", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
  auto node = rclcpp::Node::make_shared("integration_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Create a publisher for the Joy messages we're looking to test the Teleop Nodes with
  auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>("joy", 100); // puhlishing rate has to be 100

  // Create subscribers to receive all the outputs of said Teleop Nodes
  auto pntstmp_mir_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("point_stamped_mirror/desired_position", 10, pntstmp_mir_callback);
  // Will add for the other two Teleop Nodes



  rclcpp::Time start_time = rclcpp::Clock().now();

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Publish Joy message for pushing both joysticks up by 1.0
    std::vector<float> axis_input = {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<int> button_input = {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    sensor_msgs::msg::Joy joy_test_msg = joy_msg(axis_input, button_input);
    joy_pub->publish(joy_test_msg);

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(pntstmp_mir_msg.point.y == 1.0);
}

static sensor_msgs::msg::Joy joy_msg(const std::vector<float>& axis_array, const std::vector<int>& button_array)
{
  sensor_msgs::msg::Joy new_joy;
  new_joy.header.stamp = rclcpp::Clock().now();
  new_joy.axes = axis_array;
  new_joy.buttons = button_array;

  return new_joy;
}

static void pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state)
{
  pntstmp_mir_msg = pntstmp_state;
}