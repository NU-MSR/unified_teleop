/// @file
/// @brief Publishes a series of Twist commands for a robot to receive based on inputs from a control device, with the values mirroring the usage of the inputs
/// 
/// @section Publishers
///   cmd_vel (geometry_msgs/Twist) - A Twist message for robot teleoperation
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
///  `~/yaw_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the yaw value of the robot
///  `~/yaw_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the yaw value of the robot
///  `~/pitch_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the pitch value of the robot
///  `~/pitch_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the pitch value of the robot
///  `~/roll_inc (std::string) [default "UNUSED"]`      - Button assigned to increase the roll value of the robot
///  `~/roll_dec (std::string) [default "UNUSED"]`      - Button assigned to decrease the roll value of the robot
///
///  `~/x_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/y_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/z_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/yaw_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/pitch_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/roll_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_yaw_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_pitch_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_roll_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///
///  `~/x_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/y_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/z_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/yaw_flip (bool) [default false]`      -Whether the input for this movement should be flipped
///  `~/pitch_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/roll_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///
///  `~/always_enable (bool) [default false]`      - Whether control input is always enabled (USE WITH CAUTION)
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "omnid_core/parameters.h"
#include "rosnu/rosnu.hpp"

#include <string>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::string;

static geometry_msgs::msg::Twist command, p_cmd, old_p_cmd;
static sensor_msgs::msg::Joy latest_joy_state;
static bool fresh_joy_state = false;
static bool always_enable = false;
const float rate_of_change = 1.5 * std::pow(10, -5); // USER CAN ADJUST, KEEP IT EXTREMELY SMALL

float curr_x_max = 1.0;
float curr_y_max = 1.0;
float curr_z_max = 1.0;
float curr_yaw_max = 1.0;
float curr_pitch_max = 1.0;
float curr_roll_max = 1.0;

static float x_max = 1.0;
static float y_max = 1.0;
static float z_max = 1.0;
static float yaw_max = 1.0;
static float pitch_max = 1.0;
static float roll_max = 1.0;
static float alt_x_max = 0.25;
static float alt_y_max = 0.25;
static float alt_z_max = 0.25;
static float alt_yaw_max = 0.25;
static float alt_pitch_max = 0.25;
static float alt_roll_max = 0.25;
static float lin_rate_chg_fac;
static float ang_rate_chg_fac;
static bool x_flip;
static bool y_flip;
static bool z_flip;
static bool yaw_flip;
static bool pitch_flip;
static bool roll_flip;

/// @brief The input type of an input (Axis, Trigger, Button, None)
enum class InputType
{
    Axis,
    Trigger,
    Button,
    None
};

static const int UNUSED_INDEX = -1;
static const InputType UNUSED_TYPE = InputType::None;

/// @brief An object representing a particular function's (e.g. move forward, move left) input,
///        containing said input's index in the received joy message and that input's input type
class MovementInput
{
    public:
        /// @brief Index number that correlates with its position in the joy message array
        int index;
        /// @brief InputType that indicates the type of input it is (Axis, Trigger, Button, None)
        InputType type;

        /// @brief Default constructor. Initializes index to UNUSED_INDEX and type to InputType::None
        MovementInput() : index(UNUSED_INDEX), type(InputType::None) {}
        /// @brief Constructor with provided index and type parameters.
        /// @param index_no The index of the joy message array.
        /// @param input_type The type of input.
        MovementInput(int index_no, InputType input_type) : index(index_no), type(input_type) {}
};

/// @brief Returns the type of the input based on its name
///        (Axis if it begins with an 'a', Trigger if it begins with a 't', Button if it begins with a 'b', and None if the string is empty)
/// @param input_name - The name of the controller input
static InputType input_type(std::string input_name);

/// @brief Returns a MovementInput object using its input assignment name and the button mapping dictionary
/// @param input_assignment - The name of the input device's input that is used for that function
/// @param map - The button mapping based on the input device config file
static MovementInput function_input(std::string input_assignment, std::map<std::string, int> map);

/// @brief Handler for a joy message
/// @param joy_state - The state of the inputs of the controller
static void joy_callback(const sensor_msgs::msg::Joy & joy_state);

/// @brief Returns true if control inputs are enabled based on controller input
/// @param input - The controller input that will enable this function
static bool control_enabled(MovementInput input);

/// @brief Adjusts max values to set alternative values based on controller input
/// @param input - The controller input that will enable this function
static void alt_enabled(MovementInput input);

/// @brief Returns a Twist command that has zero for all of its fields
static geometry_msgs::msg::Twist zero_command();

/// @brief Based on the current/input velocity, returns velocities with an increase in x
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist x_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in x
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist x_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an increase in y
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist y_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in y
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist y_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an increase in z
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist z_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in z
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist z_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an increase in yaw
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist yaw_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in yaw
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist yaw_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an increase in pitch
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist pitch_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in pitch
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist pitch_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an increase in roll
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist roll_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input velocity, returns velocities with an decrease in roll
/// @param input - The controller input that will indicate whether the velocity changes
/// @param temp_command - The message that will be overwritten with new velocity coordinates for the robot
static geometry_msgs::msg::Twist roll_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Returns flipped velocities depenending on parameters
/// @param temp_command - The message that will be overwritten with new velocities for the robot
static geometry_msgs::msg::Twist flip_movement(geometry_msgs::msg::Twist temp_command);

static geometry_msgs::msg::Twist subtract_twist(geometry_msgs::msg::Twist subtracted, geometry_msgs::msg::Twist subtractor);

static geometry_msgs::msg::Twist normalize_twist(geometry_msgs::msg::Twist input_command, float new_mag_linear, float new_mag_angular);

static geometry_msgs::msg::Twist add_twist(geometry_msgs::msg::Twist add1, geometry_msgs::msg::Twist add2);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cmd_vel_mirror_joystick");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    // Publisher
    auto cmdvel_pos_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100); // puhlishing rate has to be 100
    
    //
    // Declaring and getting parameters
    //
    // Function -> Controller input assignments from control scheme parameters
    const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *node, "Button assigned to enable control inputs");
    const std::string alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *node, "Button assigned to activate alternative max values");
    const std::string x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *node, "Button assigned to increase the x-axis value of the robot");
    const std::string x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *node, "Button assigned to decrease the x-axis value of the robot");
    const std::string y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *node, "Button assigned to increase the y-axis value of the robot");
    const std::string y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *node, "Button assigned to decrease the y-axis value of the robot");
    const std::string z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *node, "Button assigned to increase the z-axis value of the robot");
    const std::string z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *node, "Button assigned to decrease the z-axis value of the robot");
    const std::string yaw_inc_assignment = rosnu::declare_and_get_param<std::string>("yaw_inc", "UNUSED", *node, "Button assigned to increase the yaw value of the robot");
    const std::string yaw_dec_assignment = rosnu::declare_and_get_param<std::string>("yaw_dec", "UNUSED", *node, "Button assigned to decrease the yaw value of the robot");
    const std::string pitch_inc_assignment = rosnu::declare_and_get_param<std::string>("pitch_inc", "UNUSED", *node, "Button assigned to increase the pitch value of the robot");
    const std::string pitch_dec_assignment = rosnu::declare_and_get_param<std::string>("pitch_dec", "UNUSED", *node, "Button assigned to decrease the pitch value of the robot");
    const std::string roll_inc_assignment = rosnu::declare_and_get_param<std::string>("roll_inc", "UNUSED", *node, "Button assigned to increase the roll value of the robot");
    const std::string roll_dec_assignment = rosnu::declare_and_get_param<std::string>("roll_dec", "UNUSED", *node, "Button assigned to decrease the roll value of the robot");
    // Additional parameters
    x_max = rosnu::declare_and_get_param<float>("x_max", 1.0f, *node, "The maximum output value along that axis of movement");
    y_max = rosnu::declare_and_get_param<float>("y_max", 1.0f, *node, "The maximum output value along that axis of movement");
    z_max = rosnu::declare_and_get_param<float>("z_max", 1.0f, *node, "The maximum output value along that axis of movement");
    yaw_max = rosnu::declare_and_get_param<float>("yaw_max", 1.0f, *node, "The maximum output value along that axis of movement");
    pitch_max = rosnu::declare_and_get_param<float>("pitch_max", 1.0f, *node, "The maximum output value along that axis of movement");
    roll_max = rosnu::declare_and_get_param<float>("roll_max", 1.0f, *node, "The maximum output value along that axis of movement");
    alt_x_max = rosnu::declare_and_get_param<float>("alt_x_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_y_max = rosnu::declare_and_get_param<float>("alt_y_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_z_max = rosnu::declare_and_get_param<float>("alt_z_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_yaw_max = rosnu::declare_and_get_param<float>("alt_yaw_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_pitch_max = rosnu::declare_and_get_param<float>("alt_pitch_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_roll_max = rosnu::declare_and_get_param<float>("alt_roll_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *node, "Whether the input for this movement should be flipped");
    y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *node, "Whether the input for this movement should be flipped");
    z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *node, "Whether the input for this movement should be flipped");
    yaw_flip = rosnu::declare_and_get_param<bool>("yaw_flip", false, *node, "Whether the input for this movement should be flipped");
    pitch_flip = rosnu::declare_and_get_param<bool>("pitch_flip", false, *node, "Whether the input for this movement should be flipped");
    roll_flip = rosnu::declare_and_get_param<bool>("roll_flip", false, *node, "Whether the input for this movement should be flipped");
    // Modifier parameters
    lin_rate_chg_fac = rosnu::declare_and_get_param<float>("lin_rate_chg_fac", 0.0f, *node, "Factor to the rate of change for the output's linear values");
    ang_rate_chg_fac = rosnu::declare_and_get_param<float>("ang_rate_chg_fac", 0.0f, *node, "Factor to the rate of change for the output's angular values");
    // Whether control input is ALWAYS enabled
    always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *node, "Whether control input is always enabled (USE WITH CAUTION)");
    // Getting the input device config from launch file parameters
    const std::string input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device_config", "dualshock4_mapping", *node, "Chosen input device config file");
    
    // Creating a controller input -> associated joy message index number map from the input device config file
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("unified_teleop");
    std::string full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
    YAML::Node input_device = YAML::LoadFile(full_path);
    // Getting the input device name and printing it to the serial
    const std::string device_name = input_device["name"].as<string>();
    RCLCPP_INFO(rclcpp::get_logger("cmd_vel_mirror_joystick"), ("Currently using the " + device_name + " input device").c_str());
    // Creating the button map from the input device config file
    std::map<std::string, int> button_map;
    for (const auto& it : input_device["mapping"])
    {
        button_map[it.first.as<std::string>()] = it.second.as<int>();
    }
    
    // Creating MovementInputs from the retrieved input assignments parameters and created button mapping
    MovementInput enable_input = function_input(enable_assignment, button_map);
    MovementInput alt_input = function_input(alt_assignment, button_map);
    MovementInput x_inc_input = function_input(x_inc_assignment, button_map);
    MovementInput x_dec_input = function_input(x_dec_assignment, button_map);
    MovementInput y_inc_input = function_input(y_inc_assignment, button_map);
    MovementInput y_dec_input = function_input(y_dec_assignment, button_map);
    MovementInput z_inc_input = function_input(z_inc_assignment, button_map);
    MovementInput z_dec_input = function_input(z_dec_assignment, button_map);
    MovementInput yaw_inc_input = function_input(yaw_inc_assignment, button_map);
    MovementInput yaw_dec_input = function_input(yaw_dec_assignment, button_map);
    MovementInput pitch_inc_input = function_input(pitch_inc_assignment, button_map);
    MovementInput pitch_dec_input = function_input(pitch_dec_assignment, button_map);
    MovementInput roll_inc_input = function_input(roll_inc_assignment, button_map);
    MovementInput roll_dec_input = function_input(roll_dec_assignment, button_map);

    command = zero_command();
    p_cmd = command;
    old_p_cmd = p_cmd;
    
    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            command = zero_command();

            if(control_enabled(enable_input))
            {
                // Reading the raw Twist commands
                alt_enabled(alt_input);
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

                command = flip_movement(command);

                // Processing the Twist commands with modifiers

                // Implementing rate of change modifier
                // Get the diff between curr and new
                geometry_msgs::msg::Twist diff_twist = subtract_twist(command, p_cmd);
                // Adjust the diff so that it's within the set rate_of_change
                geometry_msgs::msg::Twist adjusted_diff = normalize_twist(diff_twist, rate_of_change * lin_rate_chg_fac, rate_of_change * ang_rate_chg_fac);
                // Increment it on the new processed command
                p_cmd = add_twist(p_cmd, adjusted_diff);
            }

            old_p_cmd = p_cmd;

            cmdvel_pos_pub->publish(p_cmd);
        }
        rclcpp::spin_some(node);
    }
    return 0;
}

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_mirror_joystick"), "Provided input is empty");
        rclcpp::shutdown();
        throw std::runtime_error("Provided input is empty");
    }

    char first_character = input_name.at(0);
    
    switch (first_character)
    {
        case 'a':
            return InputType::Axis;
        case 't':
            return InputType::Trigger;
        case 'b':
            return InputType::Button;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_mirror_joystick"), "Unable to determine input type");
            rclcpp::shutdown();
            throw std::runtime_error("Unable to determine input type");
    }
    
}

static MovementInput function_input(std::string input_assignment, std::map<std::string, int> map)
{
    if (input_assignment != "UNUSED")
    {
        int index = map[input_assignment];
        return MovementInput(index, input_type(input_assignment));
    }
    else
    {
        return MovementInput();
    }
}

static bool control_enabled(const MovementInput input)
{
    if (always_enable)
    {
        return true;
    }

    return latest_joy_state.buttons.at(input.index);
}

static void alt_enabled(MovementInput input)
{
    if (input.type == InputType::None)
    {
        curr_x_max = x_max;
        curr_y_max = y_max;
        curr_z_max = z_max;
        curr_yaw_max = yaw_max;
        curr_pitch_max = pitch_max;
        curr_roll_max = roll_max;
        return;
    }
    
    float input_reading = latest_joy_state.buttons.at(input.index);
    if (input_reading == 0.0)
    {
        curr_x_max = x_max;
        curr_y_max = y_max;
        curr_z_max = z_max;
        curr_yaw_max = yaw_max;
        curr_pitch_max = pitch_max;
        curr_roll_max = roll_max;
    }
    else
    {
        curr_x_max = alt_x_max;
        curr_y_max = alt_y_max;
        curr_z_max = alt_z_max;
        curr_yaw_max = alt_yaw_max;
        curr_pitch_max = alt_pitch_max;
        curr_roll_max = alt_roll_max;
    }
}

static geometry_msgs::msg::Twist zero_command()
{
    geometry_msgs::msg::Twist new_command;

    new_command.linear.x = 0.0f;
    new_command.linear.y = 0.0f;
    new_command.linear.z = 0.0f;
    new_command.angular.x = 0.0f;
    new_command.angular.y = 0.0f;
    new_command.angular.z = 0.0f;

    return new_command;
}

static geometry_msgs::msg::Twist x_axis_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.x = curr_x_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.x = curr_x_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.x = curr_x_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist x_axis_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.x = curr_x_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.x = -1 * curr_x_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.x = -1 * curr_x_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist y_axis_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.y = curr_y_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.y = curr_y_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.y = curr_y_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist y_axis_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.y = curr_y_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.y = -1 * curr_y_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.y = -1 * curr_y_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist z_axis_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.z = curr_z_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.z = curr_z_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.z = curr_z_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist z_axis_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.linear.z = curr_z_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.linear.z = -1 * curr_z_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.linear.z = -1 * curr_z_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist yaw_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.z = curr_yaw_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.z = curr_yaw_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.z = curr_yaw_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist yaw_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.z = curr_yaw_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.z = -1 * curr_yaw_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.z = -1 * curr_yaw_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist pitch_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.y = curr_pitch_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.y = curr_pitch_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.y = curr_pitch_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist pitch_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.y = curr_pitch_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.y = -1 * curr_pitch_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.y = -1 * curr_pitch_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist roll_inc(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.x = curr_roll_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.x = curr_roll_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.x = curr_roll_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist roll_dec(const MovementInput input, geometry_msgs::msg::Twist temp_command)
{
    geometry_msgs::msg::Twist new_command = temp_command;

    float axis_reading;
    float trigger_reading;
    float button_reading;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            axis_reading = (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            if (axis_reading != 0.0f)
            {
                new_command.angular.x = curr_roll_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.angular.x = -1 * curr_roll_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.angular.x = -1 * curr_roll_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::Twist flip_movement(geometry_msgs::msg::Twist temp_command)
{
    temp_command.linear.x *= pow(-1, x_flip);
    temp_command.linear.y *= pow(-1, y_flip);
    temp_command.linear.z *= pow(-1, z_flip);
    temp_command.angular.z *= pow(-1, yaw_flip);
    temp_command.angular.y *= pow(-1, pitch_flip);
    temp_command.angular.x *= pow(-1, roll_flip);


    return temp_command;
}

static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}

static geometry_msgs::msg::Twist subtract_twist(geometry_msgs::msg::Twist subtracted, geometry_msgs::msg::Twist subtractor)
{
    geometry_msgs::msg::Twist new_command;
    new_command.linear.x = subtracted.linear.x - subtractor.linear.x;
    new_command.linear.y = subtracted.linear.y - subtractor.linear.y;
    new_command.linear.z = subtracted.linear.z - subtractor.linear.z;
    new_command.angular.x = subtracted.angular.x - subtractor.angular.x;
    new_command.angular.y = subtracted.angular.y - subtractor.angular.y;
    new_command.angular.z = subtracted.angular.z - subtractor.angular.z;
    return new_command;
}

static geometry_msgs::msg::Twist normalize_twist(geometry_msgs::msg::Twist input_command, float new_mag_linear, float new_mag_angular)
{
    geometry_msgs::msg::Twist norm_command = input_command;
    float mag_linear = sqrt(input_command.linear.x * input_command.linear.x + input_command.linear.y * input_command.linear.y + input_command.linear.z * input_command.linear.z);
    float mag_angular = sqrt(input_command.angular.x * input_command.angular.x + input_command.angular.y * input_command.angular.y + input_command.angular.z * input_command.angular.z);
    
    if (new_mag_linear == 0)
    {
        new_mag_linear = mag_linear;
    }
    if (new_mag_angular == 0)
    {
        new_mag_angular = mag_angular;
    }

    // If mag_linear != 0 adjust pos accordingly, otherwise return original vector
    if (mag_linear != 0)
    {
        norm_command.linear.x = new_mag_linear * norm_command.linear.x / mag_linear;
        norm_command.linear.y = new_mag_linear * norm_command.linear.y / mag_linear;
        norm_command.linear.z = new_mag_linear * norm_command.linear.z / mag_linear;
    }
    
    if (mag_angular != 0)
    {
        norm_command.angular.x = new_mag_angular * norm_command.angular.x / mag_angular;
        norm_command.angular.y = new_mag_angular * norm_command.angular.y / mag_angular;
        norm_command.angular.z = new_mag_angular * norm_command.angular.z / mag_angular;
    }

    return norm_command;
}

static geometry_msgs::msg::Twist add_twist(geometry_msgs::msg::Twist add1, geometry_msgs::msg::Twist add2)
{
    geometry_msgs::msg::Twist new_command;
    new_command.linear.x = add1.linear.x + add2.linear.x;
    new_command.linear.y = add1.linear.y + add2.linear.y;
    new_command.linear.z = add1.linear.z + add2.linear.z;
    new_command.angular.x = add1.angular.x + add2.angular.x;
    new_command.angular.y = add1.angular.y + add2.angular.y;
    new_command.angular.z = add1.angular.z + add2.angular.z;

    return new_command;
}