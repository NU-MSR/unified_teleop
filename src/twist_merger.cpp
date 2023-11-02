/// @file
/// @brief Publishes a series of Twist commands that combines the linear and angular components from two different Twist messages into one
/// 
/// @section Publishers
///   merged/cmd_vel (geometry_msgs/Twist) - A merged Twist message for robot teleoperation
///
/// @section Subscribers
///   linear/cmd_vel (geometry_msgs/Twist)  - A Twist message in which its linear values will be merged
///   angular/cmd_vel (geometry_msgs/Twist) - A Twist message in which its angular values will be merged

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
    auto linear_sub = node->create_subscription<geometry_msgs::msg::Twist>("linear/twist", 10, linear_callback);
    auto angular_sub = node->create_subscription<geometry_msgs::msg::Twist>("angular/twist", 10, angular_callback);

    // Publisher
    auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("merged/cmd_vel", 100);
    
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