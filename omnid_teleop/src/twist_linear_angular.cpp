/// @file
/// @brief Publishes a series of series commands for the delta robot to move based on inputs from the gamepad
/// 
/// @section Publishers
///   cmd_vel (geometry_msgs/Twist) - A commanded twist for a 2D mobile robot (all values besides linear.x, linear.y, and angular.z must be zero)
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the gamepad inputs
///
/// @section Parameters
///  `~/enable_control (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/reset_delta (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/x_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/x_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/y_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/y_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/z_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/z_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///
///  `~/x_linear_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/y_linear_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/z_linear_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "omnid_core/parameters.h"
#include "rosnu/rosnu.hpp"

#include <string>
#include <cmath>
#include <iostream>

using std::string;

static geometry_msgs::msg::Twist old_command;
static geometry_msgs::msg::Twist new_command;

/// @brief Handler for a joint state message
/// @param joints_state - The joint states of the robot
void linear_callback(const sensor_msgs::msg::JointState & joint_state);

/// @brief Handler for a joint state message
/// @param joints_state - The joint states of the robot
void angular_callback(const sensor_msgs::msg::JointState & joint_state);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("twist_linear_angular");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto linear_sub = node->create_subscription<geometry_msgs::msg::Twist>("linear/cmd_vel", 10, linear_callback);
    auto angular_sub = node->create_subscription<geometry_msgs::msg::Twist>("angular/cmd_vel", 10, angular_callback);

    // Publisher
    auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100); // puhlishing rate has to be 100, otherwise delta displays incorrect behaviour
    
    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            command = zero_command();

            if(control_enabled(enable_input))
            {
                command = x_axis_inc(x_inc_input, command);
                command = x_axis_dec(x_dec_input, command);
                command = y_axis_inc(y_inc_input, command);
                command = y_axis_dec(y_dec_input, command);
                command = z_axis_inc(z_inc_input, command);
                command = z_axis_dec(z_dec_input, command);

                command = yaw_inc(yaw_inc_input, command);
                command = yaw_dec(yaw_dec_input, command);
                command = pitch_inc(pitch_inc_input, command);
                command = pitch_dec(pitch_dec_input, command);
                command = roll_inc(roll_inc_input, command);
                command = roll_dec(roll_dec_input, command);
            }

            delta_pos_pub->publish(command);
        }
        rclcpp::spin_some(node);
    }
    return 0;
}

static void joy_callback(const geometry_msgs::msg::Twist & twsit_state)
{
    latest_joy_state = twist_state;
}