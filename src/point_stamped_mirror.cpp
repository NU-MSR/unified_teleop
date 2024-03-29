/// @file
/// @brief Publishes a series of PostStamped commands for a robot to receive based on inputs from a control device, with the values mirroring the usage of the inputs
/// 
/// @section Publishers
///   desired_position (geometry_msgs/PointStamped) - A PostStamped message for robot teleoperation
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the control device's inputs
///
/// @section Parameters
///  `~/enable_control (std::string) [default "UNUSED"]`      - Button assigned to enable control inputs
///  `~/alt_enable (std::string) [default "UNUSED"]`      - Button assigned to activate alternative max values
///
///  `~/x_axis_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the x-axis value of the robot
///  `~/x_axis_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the x-axis value of the robot
///  `~/y_axis_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the y-axis value of the robot
///  `~/y_axis_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the y-axis value of the robot
///  `~/z_axis_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the z-axis value of the robot
///  `~/z_axis_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the z-axis value of the robot
///
///  `~/x_max (double) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/y_max (double) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/z_max (double) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/alt_x_max (double) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (double) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (double) [default 0.25]`      - The alternative maximum output value along that axis of movement
///
///  `~/x_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/y_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/z_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///
///  `~/boundary_radius (double) [default 0.0]`      - Radius of the spherical space around the zero position that the robot can move in
///  `~/lin_rate_chg_fac (double) [default 0.0]`     - Factor to the rate of change for the output's values
///  `~/x_offset (double) [default 0.0]`             - The offset for the message's zero value
///  `~/y_offset (double) [default 0.0]`             - The offset for the message's zero value
///  `~/z_offset (double) [default 0.0]`             - The offset for the message's zero value
///
///  `~/always_enable (bool) [default false]`      - Whether control input is always enabled (USE WITH CAUTION)
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "unified_teleop/param_helpers.hpp"
#include "unified_teleop/pnt_stmp_modifiers.hpp"
#include "unified_teleop/controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"

#include <string>
#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <map>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointStampedMirrorNode : public rclcpp::Node
{
    public:
        PointStampedMirrorNode() :  Node("point_stamped_mirror"),
                                    new_joy_state_received(false),
                                    only_pub_with_joy(false)
        {
            //
            // PARAMETERS
            //
            // Key control node parameters
            auto pub_frequency = rosnu::declare_and_get_param<double>("frequency", 100.0f, *this, "Frequency of teleoperation output, if set to 0.0, then the node only publishes with every new joy message received");
            base_rate_of_change = rosnu::declare_and_get_param<double>("base_rate_of_change", 1.5f, *this, "Base rate of change for the output's values");
            always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *this, "Whether control input is always enabled (USE WITH CAUTION)");
            only_pub_diff_joy = rosnu::declare_and_get_param<bool>("only_pub_diff_joy", false, *this, "Whether to only publish command when a new and different joy message is received");

            // Function-to-controller input assignments
            const auto enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *this, "Button assigned to enable control inputs");
            const auto alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *this, "Button assigned to activate alternative max values");
            const auto x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *this, "Button> assigned to increase the x-axis value of the robot");
            const auto x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *this, "Button assigned to decrease the x-axis value of the robot");
            const auto y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *this, "Button assigned to increase the y-axis value of the robot");
            const auto y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *this, "Button assigned to decrease the y-axis value of the robot");
            const auto z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *this, "Button assigned to increase the z-axis value of the robot");
            const auto z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *this, "Button assigned to decrease the z-axis value of the robot");

            // Output modifier parameters
            boundary_radius = rosnu::declare_and_get_param<double>("boundary_radius", 0.0f, *this, "Radius of the spherical space around the zero position that the robot can move in");
            lin_rate_chg_fac = rosnu::declare_and_get_param<double>("lin_rate_chg_fac", 0.0f, *this, "Factor to the rate of change for the output's values, if set to 0.0, then the output will be the same as the input");
            x_max = rosnu::declare_and_get_param<double>("x_max", 1.0f, *this, "The maximum output value along that axis of movement");
            y_max = rosnu::declare_and_get_param<double>("y_max", 1.0f, *this, "The maximum output value along that axis of movement");
            z_max = rosnu::declare_and_get_param<double>("z_max", 1.0f, *this, "The maximum output value along that axis of movement");
            alt_x_max = rosnu::declare_and_get_param<double>("alt_x_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            alt_y_max = rosnu::declare_and_get_param<double>("alt_y_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            alt_z_max = rosnu::declare_and_get_param<double>("alt_z_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *this, "Whether the input for this movement should be flipped");
            y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *this, "Whether the input for this movement should be flipped");
            z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *this, "Whether the input for this movement should be flipped");
            x_offset = rosnu::declare_and_get_param<double>("x_offset", 0.0f, *this, "The offset for the message's zero value");
            y_offset = rosnu::declare_and_get_param<double>("y_offset", 0.0f, *this, "The offset for the message's zero value");
            z_offset = rosnu::declare_and_get_param<double>("z_offset", 0.0f, *this, "The offset for the message's zero value");
            
            //
            // SUBSCRIBERS & PUBLISHERS
            //
            // Subscribe to the joy topic to receive the input device's input
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PointStampedMirrorNode::joy_callback, this, _1));
            // Publish the desired position for the robot to move to
            pntstmpd_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("desired_position", 100);

            //
            // SETTING UP CONTROLLER OBJECT
            //
            // Determine the input device name for finding the appropriate config file
            const auto input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device_config", "dualshock4_mapping", *this, "Chosen input device config file");
            auto pkg_share_dir = ament_index_cpp::get_package_share_directory("unified_teleop");
            auto full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
            auto input_device = YAML::LoadFile(full_path);

            // Load Controller object w/ the input device YAML config file to load the input device's button mappings
            controller.update_button_map(input_device);
            // Set the controller's always_enabled parameter
            controller.set_always_enabled(always_enable);
            
            // Initialize MovementInputs from the input assignments parameters and controller's button mappings
            enable_input = controller.generate_MovementInput(enable_assignment);
            alt_input = controller.generate_MovementInput(alt_assignment);
            x_inc_input = controller.generate_MovementInput(x_inc_assignment);
            x_dec_input = controller.generate_MovementInput(x_dec_assignment);
            y_inc_input = controller.generate_MovementInput(y_inc_assignment);
            y_dec_input = controller.generate_MovementInput(y_dec_assignment);
            z_inc_input = controller.generate_MovementInput(z_inc_assignment);
            z_dec_input = controller.generate_MovementInput(z_dec_assignment);
            
            //
            // PREPARE FOR PUBLISHING COMMANDS
            //
            // Ensure upon start up, the robot starts in the center position
            current_command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
            previous_command = current_command;

            RCLCPP_ERROR(rclcpp::get_logger("TESTING"), "1");

            // If pub_frequency is set to 0.0, then node only publishes a message when a joy message is received - in the joy message callback
            if (pub_frequency == 0.0)
            {
                only_pub_with_joy = true;
            }
            // Otherwise, the node will publish messages at the set frequency in a timer callback
            else
            {
                timer_ = this->create_wall_timer(1.0s/pub_frequency, std::bind(&PointStampedMirrorNode::timer_callback, this));
            }
        }

    private:
        //
        // MEMBER VARIABLE DECLARATIONS
        //
        // Controller object
        rosnu::Controller controller;
        // For joy and command messages
        geometry_msgs::msg::PointStamped current_command, previous_command;
        bool new_joy_state_received;
        bool always_enable;
        double base_rate_of_change;
        bool only_pub_diff_joy;
        // For node parameters
        bool only_pub_with_joy; // Whether the node only publishes with every new received joy_state
        double x_max;
        double y_max;
        double z_max;
        double alt_x_max;
        double alt_y_max;
        double alt_z_max;
        double boundary_radius;
        double lin_rate_chg_fac;
        double x_offset;
        double y_offset;
        double z_offset;
        bool x_flip;
        bool y_flip;
        bool z_flip;
        // The various MovementInput objects to be initialized in the constructor
        std::optional<rosnu::MovementInput> enable_input;
        std::optional<rosnu::MovementInput> reset_input;
        std::optional<rosnu::MovementInput> alt_input;
        std::optional<rosnu::MovementInput> x_inc_input;
        std::optional<rosnu::MovementInput> x_dec_input;
        std::optional<rosnu::MovementInput> y_inc_input;
        std::optional<rosnu::MovementInput> y_dec_input;
        std::optional<rosnu::MovementInput> z_inc_input;
        std::optional<rosnu::MovementInput> z_dec_input;

        //
        // NODE DECLARATIONS
        //
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pntstmpd_pub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

        //
        // FUNCTIONS
        //
        /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
        void timer_callback()
        {
            // Make sure to only begin publishing if a joy message has been received
            if (new_joy_state_received)
            {
                create_modify_publish_command();
            }
        }

        /// @brief Joy callback for node, received joy_state and signals whether node should proceed with processing and publishing messages
        /// @param joy_state - The state of the inputs of the controller
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_state)
        {
            // Update the controller with the received joy state message
            controller.update_joy_state(joy_state);

            // Indicate that a new joy state has been received
            new_joy_state_received = true;

            // If the node is set to only publish with every new joy message, then the node will publish the command
            if (only_pub_with_joy)
            {
                create_modify_publish_command();
            }
        }

        void create_modify_publish_command()
        {
            // Note the current time of publishing
            rclcpp::Time current_time = rclcpp::Clock().now();
            
            //
            // READING RAW COMMAND
            //
            // Reset the raw command to zero
            current_command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
            
            // If the controller is enabled, derive the desired raw command from the controller's inputs
            if(controller.is_enabled(enable_input))
            {
                // Modifying output message based on controller's joy state
                current_command = rosnu::adjust_mirror_joy(z_inc_input, current_command, rosnu::AxisType::Z_Axis, true, controller.get_current_joy_state());
                current_command = rosnu::adjust_mirror_joy(z_dec_input, current_command, rosnu::AxisType::Z_Axis, false, controller.get_current_joy_state());
                current_command = rosnu::adjust_mirror_joy(x_inc_input, current_command, rosnu::AxisType::X_Axis, true, controller.get_current_joy_state());
                current_command = rosnu::adjust_mirror_joy(x_dec_input, current_command, rosnu::AxisType::X_Axis, false, controller.get_current_joy_state());
                current_command = rosnu::adjust_mirror_joy(y_inc_input, current_command, rosnu::AxisType::Y_Axis, true, controller.get_current_joy_state());
                current_command = rosnu::adjust_mirror_joy(y_dec_input, current_command, rosnu::AxisType::Y_Axis, false, controller.get_current_joy_state());
            }

            //
            // MODIFYING COMMAND
            //
            // Adjust the command's values based on their default or alternative max values, depending on the alt_input's state, and if enabled
            if (controller.is_enabled(enable_input))
            {
                current_command = (controller.read_MovementInput(alt_input, controller.get_current_joy_state()) > 0.0) ? 
                    rosnu::multiply_pntstmp(current_command, alt_x_max, alt_y_max, alt_z_max) : 
                    rosnu::multiply_pntstmp(current_command, x_max, y_max, z_max);
            }

            // Switch the sign of the command's values based on the x, y, and z flip parameters
            current_command = rosnu::invert_pnt_stmp(current_command, x_flip, y_flip, z_flip);

            // Limit the rate of change of the command values according to the parameters, relative previous command's values
            // NOTE that the rate of change is in the order of 10^-5 (this is arbitrary and can be adjusted as needed)
            // NOTE if lin_rate_chg_fac is set to 0.0, then the command value will experience no change to its values at all

            // Find the difference between previous and current command values
            geometry_msgs::msg::PointStamped diff_pntstmp = rosnu::subtract_pntstmp(current_command, previous_command);
            // Find an adjusted difference that has the desired magnitude/rate of change
            geometry_msgs::msg::PointStamped adjusted_diff = rosnu::normalize_pntstmp(diff_pntstmp, (base_rate_of_change * std::pow(10, -5)) * lin_rate_chg_fac);
            
            // Increment this adjusted difference to the previous command values to get the desired adjusted command values
            current_command = rosnu::add_pntstmp(previous_command, adjusted_diff);
            // Note the new adjusted command values for subsequent rate of change command calculations
            previous_command = current_command;

            // Limit the command's values to a specified spherical space around the zero position
            // NOTE if boundary_radius is set to 0.0, then the robot's command values are not constrained to a spherical space
            //      rather, they are constrained to the maximum values set by the x, y, and z max parameters

            // Check if command values are to be constrained
            if (boundary_radius != 0.0)
            {
                // Find the squared magnitude of the command's desired values from the zero position
                double distance_from_zero_sqrd = pow(current_command.point.x, 2) + pow(current_command.point.y, 2) + pow(current_command.point.z, 2);

                // If the desired squared magnitude is larger than the specified boundary radius squared,
                // adjust the magnitude of the command's magnitude so that it's within the allowed spherical space
                if (distance_from_zero_sqrd > pow(boundary_radius, 2))
                {
                    current_command = rosnu::normalize_pntstmp(current_command, boundary_radius);
                }
            }

            // Adjust the command so that the zero value of the message is offset by the x, y, and z offset parameters
            geometry_msgs::msg::PointStamped offset_cmd = rosnu::add_pntstmp(current_command, rosnu::set_pnt_stmp(x_offset, y_offset, z_offset));
            // Round the values of the message so that it does not sporadically change
            offset_cmd = rosnu::round_pntstmp(offset_cmd, 3);
            
            //
            // PUBLISHING FINALIZED COMMAND
            //
            // Check if the node is specified to only publish when a different joy message is received
            if (only_pub_diff_joy)
            {
                // If the controller's current joy state is different from the previous joy state and the controller is enabled
                // then publish the finalized command message
                if (controller.is_joy_state_different() && controller.is_enabled(enable_input))
                {
                    offset_cmd.header.stamp = current_time;
                    pntstmpd_pub->publish(offset_cmd);
                }
            }

            // Otherwise, publish the finalized command message
            else
            {
                offset_cmd.header.stamp = current_time;
                pntstmpd_pub->publish(offset_cmd);
            }
        }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointStampedMirrorNode>());
    rclcpp::shutdown();
    return 0;
}