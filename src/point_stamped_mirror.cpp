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

#include <string>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
                                    fresh_joy_state(false)
        {
            //
            // PARAMETERS
            //
            // Key control node parameters
            auto pub_frequency = rosnu::declare_and_get_param<double>("frequency", 100.0f, *this, "Frequency of teleoperation output");
            base_rate_of_change = rosnu::declare_and_get_param<double>("base_rate_of_change", 1.5f, *this, "Base rate of change for the output's values");
            always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *this, "Whether control input is always enabled (USE WITH CAUTION)");
            only_pub_diff_joy = rosnu::declare_and_get_param<bool>("only_pub_diff_joy", false, *this, "Whether to only publish command when a new and different joy message is received");

            // Function-to-controller input assignments
            const auto enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *this, "Button assigned to enable control inputs");
            const auto alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *this, "Button assigned to activate alternative max values");
            const auto x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *this, "Button assigned to increase the x-axis value of the robot");
            const auto x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *this, "Button assigned to decrease the x-axis value of the robot");
            const auto y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *this, "Button assigned to increase the y-axis value of the robot");
            const auto y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *this, "Button assigned to decrease the y-axis value of the robot");
            const auto z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *this, "Button assigned to increase the z-axis value of the robot");
            const auto z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *this, "Button assigned to decrease the z-axis value of the robot");

            // Output modifier parameters
            boundary_radius = rosnu::declare_and_get_param<double>("boundary_radius", 0.0f, *this, "Radius of the spherical space around the zero position that the robot can move in");
            lin_rate_chg_fac = rosnu::declare_and_get_param<double>("lin_rate_chg_fac", 0.0f, *this, "Factor to the rate of change for the output's values");
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
            // SUBSCRIBERS
            //
            // Subscribe to the joy topic to receive the input device's input
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PointStampedMirrorNode::joy_callback, this, _1));

            //
            // PUBLISHERS
            //
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
            // PREPARE FOR TIMER
            //
            // Ensure upon start up, the robot starts in the center position
            current_command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
            previous_command = current_command;

            // If pub_frequency is set to 0.0, then node only publishes a message when a new joy message with different input values for any of the MovementInputs is received
            // (e.g. if the node does not have button r1 mapped, then pressing it will not trigger the publishing of a new message)
            // With this, the timer will still be set to a frequency of 100.0
            if (pub_frequency == 0.0)
            {
                only_pub_with_joy = true;
                pub_frequency = 100.0;
            }
            else
            {
                only_pub_with_joy = false;
            }
            fresh_joy_state = false;

            //
            // TIMER
            //
            timer_ = this->create_wall_timer(1.0s/pub_frequency, std::bind(&PointStampedMirrorNode::timer_callback, this));
        }

    private:
        //
        // MEMBER VARIABLE DECLARATIONS
        //
        // Controller object
        rosnu::Controller controller;
        // For joy and command messages
        geometry_msgs::msg::PointStamped current_command, previous_command;
        sensor_msgs::msg::Joy latest_joy_state, previous_joy_state;
        bool fresh_joy_state;
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
        // TIMER CALLBACK
        //
        /// @brief Timer callback for node, reads joy_state to publish appropriate output messages
        void timer_callback()
        {
            create_modify_publish_command();
        }

        /// @brief Joy callback for node, received joy_state and signals whether node should proceed with processing and publishing messages
        /// @param joy_state - The state of the inputs of the controller
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_state)
        {
            previous_joy_state = latest_joy_state;
            latest_joy_state = *joy_state;
            controller.update_joy_state(joy_state);

            // If not publishing according to joy's frequency,
            // then every new joy_state is fresh and the node will publish as normal
            if (!only_pub_with_joy)
            {
                fresh_joy_state = true;
            }
            // But if node is specified to publish with each new joy message,
            // then only if the relevant inputs of joy_state are changed AND with control enabled
            // will the joy_state be deemed fresh and ready for node to publish a new output message
            else
            {
                if (controller.is_joy_state_different() && controller.is_enabled(enable_input))
                {
                    fresh_joy_state = true;
                }
                else
                {
                    fresh_joy_state = false;
                }
            }
        }

        void create_modify_publish_command()
        {
            rclcpp::Time current_time = rclcpp::Clock().now();
            
            //
            // READING RAW COMMAND
            //
            // If a fresh_joy_state has arrived, the raw command will be adjusted
            if(fresh_joy_state)
            {
                // If the node has control enabled, derive the desired command from the inputs
                if(controller.is_enabled(enable_input))
                {
                    current_command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);

                    // Modifying output message based on joy_state
                    current_command = rosnu::adjust_mirror_joy(z_inc_input, current_command, rosnu::AxisType::Z_Axis, true, controller.get_current_joy_state());
                    current_command = rosnu::adjust_mirror_joy(z_dec_input, current_command, rosnu::AxisType::Z_Axis, false, controller.get_current_joy_state());
                    current_command = rosnu::adjust_mirror_joy(x_inc_input, current_command, rosnu::AxisType::X_Axis, true, controller.get_current_joy_state());
                    current_command = rosnu::adjust_mirror_joy(x_dec_input, current_command, rosnu::AxisType::X_Axis, false, controller.get_current_joy_state());
                    current_command = rosnu::adjust_mirror_joy(y_inc_input, current_command, rosnu::AxisType::Y_Axis, true, controller.get_current_joy_state());
                    current_command = rosnu::adjust_mirror_joy(y_dec_input, current_command, rosnu::AxisType::Y_Axis, false, controller.get_current_joy_state());

                    current_command = (controller.read_MovementInput(alt_input, controller.get_current_joy_state()) > 0.0) ? 
                        rosnu::multiply_pntstmp(current_command, alt_x_max, alt_y_max, alt_z_max) : 
                        rosnu::multiply_pntstmp(current_command, x_max, y_max, z_max);

                    current_command = rosnu::invert_pnt_stmp(current_command, x_flip, y_flip, z_flip); // Adjusting raw command based on parameters
                }
                // If control is not enabled, then raw command is set to the zero position  
                else
                {
                    current_command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
                }
            }

            //
            // APPLYING MODIFIERS
            //
            // Modifiers will always be applied to raw commands
            // Implementing rate of change modifier
            // Get the diff between curr and new
            geometry_msgs::msg::PointStamped diff_pntstmp = rosnu::subtract_pntstmp(current_command, previous_command);
            // Adjust the diff so that it's within the set base_rate_of_change
            geometry_msgs::msg::PointStamped adjusted_diff = rosnu::normalize_pntstmp(diff_pntstmp, (base_rate_of_change * std::pow(10, -5)) * lin_rate_chg_fac);
            // Increment it on the new processed command
            previous_command = rosnu::add_pntstmp(previous_command, adjusted_diff);

            // Implementing spherical positional boundary modifier
            // Make sure the robot's position is constrained to the desired spherical space
            if (boundary_radius != 0.0)
            {
                // Find the robot's desired distance from home sqrd
                double distance_from_home_sqrd = pow(previous_command.point.x, 2) + pow(previous_command.point.y, 2) + pow(previous_command.point.z, 2);
                // If the distance is larger than the desired boundary radius, normalize the position's magnitude so that it's within allowed space
                if (distance_from_home_sqrd > pow(boundary_radius, 2))
                {
                    previous_command = rosnu::normalize_pntstmp(previous_command, boundary_radius);
                }
            }

            // Adjust command so that it is offset as desired
            geometry_msgs::msg::PointStamped offset_cmd = rosnu::add_pntstmp(previous_command, rosnu::set_pnt_stmp(x_offset, y_offset, z_offset));
            // Round the values of the message so that it does not sporadically change
            offset_cmd = rosnu::round_pntstmp(offset_cmd, 3);
            
            //
            // PUBLISHING MODIFIED COMMAND
            //
            // Node will always publish if the frequency is not only_pub_with_joy
            if (!only_pub_with_joy)
            {
                offset_cmd.header.stamp = current_time;
                pntstmpd_pub->publish(offset_cmd);
            }
            // But if publishing frequency is set to only_pub_with_joy, then it only publishes
            // when a fresh_joy_state has been received
            else
            {
                if (fresh_joy_state)
                {
                    offset_cmd.header.stamp = current_time;
                    pntstmpd_pub->publish(offset_cmd);
                    fresh_joy_state = false; // Need to wait for the next fresh_joy_state to publish the next command
                }
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