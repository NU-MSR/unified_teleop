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
// Saved messages from the different teleop nodes
geometry_msgs::msg::Point node_pntstmp_mir_point, bag_pntstmp_mir_point;
// Vectors that will contain the list of output messages from ros2 bag and the tested teleop node
std::vector<geometry_msgs::msg::Point> node_point_vec, bag_point_vec;

/// @brief Handler for a point_stamped message from the tested teleop node
/// @param pntstmp_state - The state of the desired point_stamped output
static void node_pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state);

/// @brief Handler for a point_stamped message from ROS2 bag
/// @param pntstmp_state - The state of the desired point_stamped output
static void bag_pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state);

/// @brief 
/// @param pntstmp_state - 
static bool is_nearby(const geometry_msgs::msg::Point & pnt_a, const geometry_msgs::msg::Point & pnt_b, const double radius);

// /// @brief Handler for a joy message from ROS2 bag
// /// @param joy_state - The state of the inputs of the controller
// static void bag_joy_callback(const geometry_msgs::msg::PointStamped & pntstmp_state);

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

  // // Create a publisher for the Joy messages we're looking to test the teleop nodes with
  // auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>("joy", 100); // puhlishing rate has to be 100

  //
  // SUBSCRIBERS
  //
  // Create subscriber to receive point_stamped output of teleop node
  auto node_pntstmp_mir_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("point_stamped_mirror/desired_position", 10, node_pntstmp_mir_callback);
  // Create subscriber to receive point_stamped output of ros2 bag
  auto bag_pntstmp_mir_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("ros2_bag/desired_position", 10, bag_pntstmp_mir_callback);

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

  // // Resize the node_point_vec vector to allow for comparison with the bag_point_vec vector
  // int bag_point_vec_size = bag_point_vec.size();
  // node_point_vec.resize(bag_point_vec_size);

  int vec_size = bag_point_vec.size();
  bag_point_vec.resize(vec_size);
  node_point_vec.resize(vec_size);

  RCLCPP_INFO(node->get_logger(), "bag point vec size: '%ld'", bag_point_vec.size());

  // geometry_msgs::msg::Point test_node, test_bag;
  // int index = 10000;
  // test_bag = bag_point_vec[index];
  // test_node = node_point_vec[index];
  // RCLCPP_INFO(
  //     node->get_logger(),
  //     "Bag coordinates: x = %f, y = %f, z = %f",
  //     test_bag.x, test_bag.y, test_bag.z
  // );
  // RCLCPP_INFO(
  //     node->get_logger(),
  //     "Bag coordinates: x = %f, y = %f, z = %f",
  //     test_node.x, test_node.y, test_node.z
  // );

  // // Test assertions - check that the expected output from ros2 bag and the tested output from the teleop node matches
  // // Cannot directly compare both vectors due to unique data type of ros2 messages
  // for (int i = 0; i < bag_point_vec.size(); i++)
  // {
  //   if ((i % 40) == 0)
  //   {
  //     geometry_msgs::msg::Point test_node, test_bag;
  //     int index = i;
  //     test_bag = bag_point_vec[index];
  //     test_node = node_point_vec[index];
  //     RCLCPP_INFO(
  //         node->get_logger(),
  //         "Bag coordinates: x = %f, y = %f, z = %f",
  //         test_bag.x, test_bag.y, test_bag.z
  //     );
  //     RCLCPP_INFO(
  //         node->get_logger(),
  //         "Node coordinates: x = %f, y = %f, z = %f",
  //         test_node.x, test_node.y, test_node.z
  //     );
  //   }
  // }

  // Test assertions - check that the expected output from ros2 bag and the tested output from the teleop node matches
  // Cannot directly compare both vectors due to unique data type of ros2 messages
  int proximity_count = 0;
  for (int i = 0; i < static_cast<int>(bag_point_vec.size()); i++)
  {
    // Cannot directly compare one-to-one due to slightly undeterministic behaviour of ros2 nodes
    // Check if expected and tested outputs are within a user-defined radius/proximity of one another
    RCLCPP_INFO(node->get_logger(), "bag point: '%f' '%f' '%f'", bag_point_vec[i].x, bag_point_vec[i].y, bag_point_vec[i].z);
    RCLCPP_INFO(node->get_logger(), "node point: '%f' '%f' '%f'", node_point_vec[i].x, node_point_vec[i].y, node_point_vec[i].z);
    if (is_nearby(bag_point_vec[i], node_point_vec[i], 0.05))
    {
      proximity_count += 1;
    }
  }
  // Compute the proximity rate between expected and tested outputs
  // Check if proximity rate is above a user-defined percentage to decide if overall expected and tested behaviours are similar enough
  double proximity_rate = (proximity_count/bag_point_vec.size()) * 100;
  RCLCPP_INFO(node->get_logger(), "bag point vec size: '%ld'", bag_point_vec.size());
  RCLCPP_INFO(
          node->get_logger(),
          "Success Rate: %f",
          proximity_rate
        );
  bool test_success = (proximity_rate >= 90.0);
  RCLCPP_INFO(node->get_logger(), "Completed comparing message vectors");
  CHECK(test_success);

  // RCLCPP_INFO(node->get_logger(), "Completed comparing message vectors");
  // bool test_success = (node_point_vec == bag_point_vec);
  // CHECK(test_success);
}

static void node_pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state)
{
  // std::cout << "node callback started";
  node_pntstmp_mir_point = pntstmp_state.point;
  // Append the new message to a vector
  node_point_vec.push_back(node_pntstmp_mir_point);
}

static void bag_pntstmp_mir_callback(const geometry_msgs::msg::PointStamped & pntstmp_state)
{
  // std::cout << "bag callback started";
  bag_pntstmp_mir_point = pntstmp_state.point;
  // Append the new message to a vector
  bag_point_vec.push_back(bag_pntstmp_mir_point);
}

static bool is_nearby(const geometry_msgs::msg::Point & pnt_a, const geometry_msgs::msg::Point & pnt_b, const double radius)
{
  float distance = pow(pnt_a.x - pnt_b.x, 2) + pow(pnt_a.y - pnt_b.y, 2) + pow(pnt_a.z - pnt_b.z, 2);
  return distance <= pow(radius, 2);
}