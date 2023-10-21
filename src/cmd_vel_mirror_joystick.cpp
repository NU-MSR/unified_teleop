/// NEED TO UPDATE DOCUMENTATAION HERE

/// @file
/// @brief Publishes a series of cmd_vel commands for a robot to receive based on inputs from a control device
/// 
/// @section Publishers
///   cmd_vel (geometry_msgs/Twist) - A commanded twist message for the robot
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the control device's inputs
///
/// @section Parameters
///  `~/enable_control (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/alt_enable (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/reset_cmdvel_ (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/x_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/x_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/y_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/y_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/z_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/z_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/yaw_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/yaw_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/pitch_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/pitch_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/roll_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///  `~/roll_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control this function
///
///  `~/x_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of movement
///  `~/y_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of movement
///  `~/z_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of movement
///  `~/yaw_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of rotation
///  `~/pitch_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of rotation
///  `~/roll_max (float) [default 1.0]`      - The maximum Twist value for movement speed along that axis of rotation
///  `~/x_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of movement
///  `~/y_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of movement
///  `~/z_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of movement
///  `~/yaw_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of rotation
///  `~/pitch_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of rotation
///  `~/roll_max (float) [default 1.0]`      - The alternative maximum Twist value for movement speed along that axis of rotation
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

static geometry_msgs::msg::Twist command;
static sensor_msgs::msg::Joy latest_joy_state;
static bool fresh_joy_state = false;
static bool always_enable = false;

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

/// @brief The type of input a particular device input can be
enum class InputType
{
    Axis,
    Trigger,
    Button,
    None
};

static const int UNUSED_INDEX = -1;
static const InputType UNUSED_TYPE = InputType::None;

/// @brief An object representing a particular function's (e.g. move forward, move left) input, containing said input's index in the joy message and that input's input type
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

/// @brief Returns the type of the input based on its name (Axis, Trigger, Button, or None)
/// @param input_name - The name of the controller input
static InputType input_type(std::string input_name);

/// @brief Returns a MovementInput object using its input assignment name and the button mapping dictionary
/// @param input_assignment - The name of the input device's input that is used for that function
/// @param map - The button mapping based on the input device config file
static MovementInput function_input(std::string input_assignment, std::map<std::string, int> map);

/// @brief Handler for a joy message
/// @param joy_state - The states of the inputs of the controller
static void joy_callback(const sensor_msgs::msg::Joy & joy_state);

/// @brief Indicates whether delta movement has been enabled based on controller input
/// @param input - The controller input that will indicate whether the delta is enabled
static bool control_enabled(MovementInput input);

/// @brief Indicates whether alternative movement max values has been enabled based on controller input
/// @param input - The controller input that will indicate whether this function is activated
static void alt_enabled(MovementInput input);

/// @brief Returns a Twist command that has zero for all of its fields
static geometry_msgs::msg::Twist zero_command();

/// @brief Based on the current/input position, returns coords that move the delta's position forward
/// @param input - The controller input that will indicate whether the delta will move forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist x_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position backward
/// @param input - The controller input that will indicate whether the delta will move backward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist x_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position left
/// @param input - The controller input that will indicate whether the delta will move left
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist y_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position right
/// @param input - The controller input that will indicate whether the delta will move right
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist y_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position up
/// @param input - The controller input that will indicate whether the delta will move up
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist z_axis_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position down
/// @param input - The controller input that will indicate whether the delta will move down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist z_axis_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position forward
/// @param input - The controller input that will indicate whether the delta will move forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist yaw_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position backward
/// @param input - The controller input that will indicate whether the delta will move backward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist yaw_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position left
/// @param input - The controller input that will indicate whether the delta will move left
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist pitch_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position right
/// @param input - The controller input that will indicate whether the delta will move right
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist pitch_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position up
/// @param input - The controller input that will indicate whether the delta will move up
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist roll_inc(MovementInput input, geometry_msgs::msg::Twist temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position down
/// @param input - The controller input that will indicate whether the delta will move down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::Twist roll_dec(MovementInput input, geometry_msgs::msg::Twist temp_command);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cmd_vel_mirror_joystick");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    // Publisher
    auto cmdvel_pos_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100); // puhlishing rate has to be 100, otherwise delta displays incorrect behaviour
    
    //
    // Declaring and getting parameters
    //
    // Function -> Controller input assignments from control scheme parameters
    const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *node, "Button assigned to enable delta movement");
    const std::string alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *node, "Button assigned to activate alternative max values");
    const std::string x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *node, "Button assigned to move robot forward");
    const std::string x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *node, "Button assigned to move robot backward");
    const std::string y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *node, "Button assigned to move robot left");
    const std::string y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *node, "Button assigned to move robot right");
    const std::string z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *node, "Button assigned to move robot up");
    const std::string z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *node, "Button assigned to move robot down");
    const std::string yaw_inc_assignment = rosnu::declare_and_get_param<std::string>("yaw_inc", "UNUSED", *node, "Button assigned to increase the yaw of the robot");
    const std::string yaw_dec_assignment = rosnu::declare_and_get_param<std::string>("yaw_dec", "UNUSED", *node, "Button assigned to decrease the yaw of the robot");
    const std::string pitch_inc_assignment = rosnu::declare_and_get_param<std::string>("pitch_inc", "UNUSED", *node, "Button assigned to increase the pitch of the robot");
    const std::string pitch_dec_assignment = rosnu::declare_and_get_param<std::string>("pitch_dec", "UNUSED", *node, "Button assigned to decrease the pitch of the robot");
    const std::string roll_inc_assignment = rosnu::declare_and_get_param<std::string>("roll_inc", "UNUSED", *node, "Button assigned to increase the roll of the robot");
    const std::string roll_dec_assignment = rosnu::declare_and_get_param<std::string>("roll_dec", "UNUSED", *node, "Button assigned to decrease the roll of the robot");
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
    // Whether control input is ALWAYS enabled (USE WITH CAUTION)
    always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *node, "Whether control input is always enabled");
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
    // Enabling Delta movement
    MovementInput enable_input = function_input(enable_assignment, button_map);
    // Enabling Delta movement
    MovementInput alt_input = function_input(alt_assignment, button_map);
    // Moving Delta forward
    MovementInput x_inc_input = function_input(x_inc_assignment, button_map);
    // Moving Delta backward
    MovementInput x_dec_input = function_input(x_dec_assignment, button_map);
    // Moving Delta left
    MovementInput y_inc_input = function_input(y_inc_assignment, button_map);
    // Moving Delta right
    MovementInput y_dec_input = function_input(y_dec_assignment, button_map);
    // Moving Delta up
    MovementInput z_inc_input = function_input(z_inc_assignment, button_map);
    // Moving Delta down
    MovementInput z_dec_input = function_input(z_dec_assignment, button_map);
    //
    MovementInput yaw_inc_input = function_input(yaw_inc_assignment, button_map);
    // Moving Delta backward
    MovementInput yaw_dec_input = function_input(yaw_dec_assignment, button_map);
    // Moving Delta left
    MovementInput pitch_inc_input = function_input(pitch_inc_assignment, button_map);
    // Moving Delta right
    MovementInput pitch_dec_input = function_input(pitch_dec_assignment, button_map);
    // Moving Delta up
    MovementInput roll_inc_input = function_input(roll_inc_assignment, button_map);
    // Moving Delta down
    MovementInput roll_dec_input = function_input(roll_dec_assignment, button_map);

    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            command = zero_command();

            // RCLCPP_INFO(node->get_logger(), "HERE 1");

            if(control_enabled(enable_input))
            {
                // RCLCPP_INFO(node->get_logger(), "HERE 2");
                alt_enabled(alt_input);
                // RCLCPP_INFO(node->get_logger(), "HERE 3");
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

            cmdvel_pos_pub->publish(command);
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

static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}