/// NEED TO UPDATE DOCUMENTATION HERE

/// @file
/// @brief Publishes a series of series commands for the delta robot to move based on inputs from the gamepad
/// 
/// @section Publishers
///   cmd_vel (geometry_msgs/Twist) - A commanded twist for a 2D mobile robot (all values besides linear.x, linear.y, and angular.z must be zero)
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the gamepad inputs

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <cmath>
#include <iostream>

using std::string;

static geometry_msgs::msg::Twist latest_linear;
static geometry_msgs::msg::Twist latest_angular;
static geometry_msgs::msg::Twist old_command;
static geometry_msgs::msg::Twist new_command;

/// @brief Handler for a joint state message
/// @param joints_state - The joint states of the robot
static void linear_callback(const geometry_msgs::msg::Twist & twist_state);

/// @brief Handler for a joint state message
/// @param joints_state - The joint states of the robot
static void angular_callback(const geometry_msgs::msg::Twist & twist_state);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("twist_merger");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto linear_sub = node->create_subscription<geometry_msgs::msg::Twist>("linear/cmd_vel", 10, linear_callback);
    auto angular_sub = node->create_subscription<geometry_msgs::msg::Twist>("angular/cmd_vel", 10, angular_callback);

    // Publisher
    auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100); // puhlishing rate has to be 100, otherwise delta displays incorrect behaviour
    
    // Control loop
    while (rclcpp::ok())
    {
        new_command = old_command;

        new_command.linear.x = latest_linear.linear.x;
        new_command.linear.y = latest_linear.linear.y;
        new_command.linear.z = latest_linear.linear.z;

        new_command.angular.x = latest_angular.angular.x;
        new_command.angular.y = latest_angular.angular.y;
        new_command.angular.z = latest_angular.angular.z;

        twist_pub->publish(new_command);
        
        rclcpp::spin_some(node);
    }
    return 0;
}

static void linear_callback(const geometry_msgs::msg::Twist & twist_state)
{
    latest_linear = twist_state;
}

static void angular_callback(const geometry_msgs::msg::Twist & twist_state)
{
    latest_angular = twist_state;
}