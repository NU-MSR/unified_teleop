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

#include <vector>
#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

// FOR TESTING
#include <iostream>

using namespace std::chrono_literals;

//
// GLOBAL VARIABLES
//
// Saved messages from the callbacks
sensor_msgs::msg::Joy new_joy;
// Vectors that will contain the list of output messages from ros2 bag and the tested teleop node
std::vector<sensor_msgs::msg::Joy> joy_vec;

/// @brief Handler for a point_stamped message from ROS2 bag
/// @param pntstmp_state - The state of the desired point_stamped output
static void bag_joy_callback(const sensor_msgs::msg::Joy & joy_state);

TEST_CASE("integration_test", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
  auto node = rclcpp::Node::make_shared("integration_test_node");

  //
  // VARIABLES
  //

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  //
  // SUBSCRIBERS
  //
  // Create subscriber to receive joy output of ros2 bag
  auto bag_joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bag_joy_callback);

  rclcpp::Time start_time = rclcpp::Clock().now();

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    rclcpp::spin_some(node); // Do nothing and let the callbacks collect the expected and tested outputs
  }
  RCLCPP_INFO(node->get_logger(), "Finished waiting");
  RCLCPP_INFO(node->get_logger(), "joy vec size: '%ld'", joy_vec.size());
  bool test_success = (joy_vec.size() == 20);
  CHECK(test_success);
}

static void bag_joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
  // std::cout << "bag callback started";
  new_joy = joy_state;
  // Append the new message to a vector
  joy_vec.push_back(new_joy);
}