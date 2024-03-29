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
        //
        // CONSTRUCTOR
        //
        PointStampedMirrorNode() :  Node("point_stamped_mirror"),
                                    fresh_joy_state(false),
                                    always_enable(false),
                                    rate_of_change(1.5 * std::pow(10, -5))
        {
            //
            // PARAMETERS
            //
            // Frequency of publisher
            auto pub_frequency = rosnu::declare_and_get_param<double>("frequency", 100.0f, *this, "Frequency of teleoperation output");
            // Function -> Controller input assignments from control scheme parameters
            const auto enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *this, "Button assigned to enable control inputs");
            const auto alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *this, "Button assigned to activate alternative max values");
            const auto x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *this, "Button assigned to increase the x-axis value of the robot");
            const auto x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *this, "Button assigned to decrease the x-axis value of the robot");
            const auto y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *this, "Button assigned to increase the y-axis value of the robot");
            const auto y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *this, "Button assigned to decrease the y-axis value of the robot");
            const auto z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *this, "Button assigned to increase the z-axis value of the robot");
            const auto z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *this, "Button assigned to decrease the z-axis value of the robot");
            // Additional parameters
            x_max = rosnu::declare_and_get_param<double>("x_max", 1.0f, *this, "The maximum output value along that axis of movement");
            y_max = rosnu::declare_and_get_param<double>("y_max", 1.0f, *this, "The maximum output value along that axis of movement");
            z_max = rosnu::declare_and_get_param<double>("z_max", 1.0f, *this, "The maximum output value along that axis of movement");
            alt_x_max = rosnu::declare_and_get_param<double>("alt_x_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            alt_y_max = rosnu::declare_and_get_param<double>("alt_y_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            alt_z_max = rosnu::declare_and_get_param<double>("alt_z_max", 0.25f, *this, "The alternative maximum output value along that axis of movement");
            x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *this, "Whether the input for this movement should be flipped");
            y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *this, "Whether the input for this movement should be flipped");
            z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *this, "Whether the input for this movement should be flipped");
            // Modifier parameters
            boundary_radius = rosnu::declare_and_get_param<double>("boundary_radius", 0.0f, *this, "Radius of the spherical space around the zero position that the robot can move in");
            lin_rate_chg_fac = rosnu::declare_and_get_param<double>("lin_rate_chg_fac", 0.0f, *this, "Factor to the rate of change for the output's values");
            x_offset = rosnu::declare_and_get_param<double>("x_offset", 0.0f, *this, "The offset for the message's zero value");
            y_offset = rosnu::declare_and_get_param<double>("y_offset", 0.0f, *this, "The offset for the message's zero value");
            z_offset = rosnu::declare_and_get_param<double>("z_offset", 0.0f, *this, "The offset for the message's zero value");
            // Whether control input is ALWAYS enabled
            always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *this, "Whether control input is always enabled (USE WITH CAUTION)");
            
            //
            // SUBSCRIBERS
            //
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PointStampedMirrorNode::joy_callback, this, _1));

            //
            // PUBLISHERS
            //
            pntstmpd_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("desired_position", 100);

            //
            // INTEGRATING INPUT & OUTPUT SCHEMES
            //
            // Getting the input device config from launch file parameters
            const auto input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device_config", "dualshock4_mapping", *this, "Chosen input device config file");
            // Creating a controller input -> associated joy message index number map from the input device config file
            auto pkg_share_dir = ament_index_cpp::get_package_share_directory("unified_teleop");
            auto full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
            auto input_device = YAML::LoadFile(full_path);
            // Load Controller object w/ the input device YAML config file
            controller.update_button_map(input_device);
            
            // Initializing MovementInputs from the retrieved input assignments parameters and created button mapping
            enable_input = controller.generate_MovementInput(enable_assignment);
            alt_input = controller.generate_MovementInput(alt_assignment);
            x_inc_input = controller.generate_MovementInput(x_inc_assignment);
            x_dec_input = controller.generate_MovementInput(x_dec_assignment);
            y_inc_input = controller.generate_MovementInput(y_inc_assignment);
            y_dec_input = controller.generate_MovementInput(y_dec_assignment);
            z_inc_input = controller.generate_MovementInput(z_inc_assignment);
            z_dec_input = controller.generate_MovementInput(z_dec_assignment);
            // Store all initialized MovementInputs in a vector
            move_input_vec = {enable_input, alt_input, x_inc_input, x_dec_input,
                                y_inc_input, y_dec_input, z_inc_input, z_dec_input};

            //
            // INITIALIZING VARIABLES
            //  
            is_first_joy = true;
            
            // Ensure upon start up, the robot starts in the center position
            command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
            p_cmd = command;
            // If pub_frequency is set to 0.0, then node only publishes a message when a new joy message
            // with different input values for any of the MovementInputs is received
            // (e.g. if the node does not have button r1 mapped, then pressing it will not trigger the publishing of a new message)
            // With this, the timer will still be set to a frequency of 100.0
            if (pub_frequency == 0.0)
            {
                is_joy_freq = true;
                pub_frequency = 100.0;
            }
            else
            {
                is_joy_freq = false;
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
        geometry_msgs::msg::PointStamped command, p_cmd;
        sensor_msgs::msg::Joy latest_joy_state, previous_joy_state;
        bool fresh_joy_state;
        bool always_enable;
        const double rate_of_change;
        // For node parameters
        bool is_first_joy; // Whether the received joy_state is the first one
        bool is_joy_freq; // Whether the node only publishes with every new received joy_state
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
        std::vector<std::optional<rosnu::MovementInput>> move_input_vec; // Vector containing all initialized MovementInput objects

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
                    command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
                    
                    alt_enabled(alt_input); // Adjusting max values according to parameters

                    // Modifying output message based on joy_state
                    command = rosnu::adjust_mirror_joy(z_inc_input, command, rosnu::AxisType::Z_Axis, true, controller.get_current_joy_state());
                    command = rosnu::adjust_mirror_joy(z_dec_input, command, rosnu::AxisType::Z_Axis, false, controller.get_current_joy_state());
                    command = rosnu::adjust_mirror_joy(x_inc_input, command, rosnu::AxisType::X_Axis, true, controller.get_current_joy_state());
                    command = rosnu::adjust_mirror_joy(x_dec_input, command, rosnu::AxisType::X_Axis, false, controller.get_current_joy_state());
                    command = rosnu::adjust_mirror_joy(y_inc_input, command, rosnu::AxisType::Y_Axis, true, controller.get_current_joy_state());
                    command = rosnu::adjust_mirror_joy(y_dec_input, command, rosnu::AxisType::Y_Axis, false, controller.get_current_joy_state());

                    command = (controller.read_MovementInput(alt_input, controller.get_current_joy_state()) > 0.0) ? 
                        rosnu::multiply_pntstmp(command, alt_x_max, alt_y_max, alt_z_max) : 
                        rosnu::multiply_pntstmp(command, x_max, y_max, z_max);

                    command = rosnu::invert_pnt_stmp(command, x_flip, y_flip, z_flip); // Adjusting raw command based on parameters
                }
                // If control is not enabled, then raw command is set to the zero position  
                else
                {
                    command = rosnu::set_pnt_stmp(0.0, 0.0, 0.0);
                }
            }

            //
            // APPLYING MODIFIERS
            //
            // Modifiers will always be applied to raw commands
            // Implementing rate of change modifier
            // Get the diff between curr and new
            geometry_msgs::msg::PointStamped diff_pntstmp = rosnu::subtract_pntstmp(command, p_cmd);
            // Adjust the diff so that it's within the set rate_of_change
            geometry_msgs::msg::PointStamped adjusted_diff = rosnu::normalize_pntstmp(diff_pntstmp, rate_of_change * lin_rate_chg_fac);
            // Increment it on the new processed command
            p_cmd = rosnu::add_pntstmp(p_cmd, adjusted_diff);

            // Implementing spherical positional boundary modifier
            // Make sure the robot's position is constrained to the desired spherical space
            if (boundary_radius != 0.0)
            {
                // Find the robot's desired distance from home sqrd
                double distance_from_home_sqrd = pow(p_cmd.point.x, 2) + pow(p_cmd.point.y, 2) + pow(p_cmd.point.z, 2);
                // If the distance is larger than the desired boundary radius, normalize the position's magnitude so that it's within allowed space
                if (distance_from_home_sqrd > pow(boundary_radius, 2))
                {
                    p_cmd = rosnu::normalize_pntstmp(p_cmd, boundary_radius);
                }
            }

            // Adjust command so that it is offset as desired
            geometry_msgs::msg::PointStamped offset_cmd = rosnu::add_pntstmp(p_cmd, rosnu::set_pnt_stmp(x_offset, y_offset, z_offset));
            // Round the values of the message so that it does not sporadically change
            offset_cmd = rosnu::round_pntstmp(offset_cmd, 3);
            
            //
            // PUBLISHING MODIFIED COMMAND
            //
            // Node will always publish if the frequency is not is_joy_freq
            if (!is_joy_freq)
            {
                offset_cmd.header.stamp = current_time;
                pntstmpd_pub->publish(offset_cmd);
            }
            // But if publishing frequency is set to is_joy_freq, then it only publishes
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

        //
        // SUBSCRIBER CALLBACKS
        //
        /// @brief Joy callback for node, received joy_state and signals whether node should proceed with processing and publishing messages
        /// @param joy_state - The state of the inputs of the controller
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_state)
        {
            // RCLCPP_INFO(rclcpp::get_logger("point_stamped_incr"), "TEST JOY");
            previous_joy_state = latest_joy_state;
            latest_joy_state = *joy_state;
            controller.update_joy_state(joy_state);
            // If received joy_state is the first one, then assign same joy_state to previous_joy_state
            if (is_first_joy)
            {
                is_first_joy = false;
                previous_joy_state = latest_joy_state;
            }

            // If not publishing according to joy's frequency,
            // then every new joy_state is fresh and the node will publish as normal
            if (!is_joy_freq)
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
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointStampedMirrorNode>());
    rclcpp::shutdown();
    return 0;
}