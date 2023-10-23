/// @file
/// @brief Publishes a series of series commands for the delta robot to move based on inputs from the gamepad
/// 
/// @section Publishers
///   desired_position (geometry_msgs/PointStamped) - The desired position of the end effector
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the gamepad inputs
///
/// @section Parameters
///  `~/enable_control (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/reset_pntstmpd (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/x_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/x_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/y_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/y_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/z_axis_inc (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/z_axis_dec (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///
///  `~/curr_x_incr_mult (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/curr_y_incr_mult (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/curr_z_incr_mult (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///
///  `~/input_device (std::string) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "omnid_core/parameters.h"
#include "rosnu/rosnu.hpp"

#include <string>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::string;

static geometry_msgs::msg::PointStamped command;
static sensor_msgs::msg::Joy latest_joy_state;
static bool fresh_joy_state = false;
static bool always_enable = false;

float curr_x_max = 1.0;
float curr_y_max = 1.0;
float curr_z_max = 1.0;
static float x_max;
static float y_max;
static float z_max;
static float alt_x_max;
static float alt_y_max;
static float alt_z_max;
static bool x_flip;
static bool y_flip;
static bool z_flip;

static const float incr_denom = 1 * std::pow(10, 5); // increment value chosen arbitrarily
static float incr_mult = 1.0;

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

/// @brief Based on the current/input position, returns coords that move the delta's position forward
/// @param input - The controller input that will indicate whether the delta will move forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped axes_reset(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Sets a given input command to contain a given position
/// @param input_command - The message that will be overwritten with center position coordinates for the delta
static geometry_msgs::msg::PointStamped zero_command();

/// @brief Based on the current/input position, returns coords that move the delta's position forward
/// @param input - The controller input that will indicate whether the delta will move forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped x_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position backward
/// @param input - The controller input that will indicate whether the delta will move backward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped x_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position left
/// @param input - The controller input that will indicate whether the delta will move left
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped y_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position right
/// @param input - The controller input that will indicate whether the delta will move right
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped y_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position up
/// @param input - The controller input that will indicate whether the delta will move up
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped z_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position down
/// @param input - The controller input that will indicate whether the delta will move down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped z_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords that move the delta's position down
/// @param input - The controller input that will indicate whether the delta will move down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static geometry_msgs::msg::PointStamped flip_movement(geometry_msgs::msg::PointStamped temp_command);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("point_stamped_incr_joystick");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    // Publisher
    auto pntstmpd_pos_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("desired_position", 100); // puhlishing rate has to be 100, otherwise delta displays incorrect behaviour
    
    //
    // Declaring and getting parameters
    //
    // Function -> Controller input assignments from control scheme parameters
    const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *node, "Button assigned to enable delta movement");
    const std::string reset_assignment = rosnu::declare_and_get_param<std::string>("reset_enable", "UNUSED", *node, "Button assigned to reset delta position");
    const std::string alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *node, "Button assigned to activate alternative max values");
    const std::string x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *node, "Button assigned to move robot forward");
    const std::string x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *node, "Button assigned to move robot backward");
    const std::string y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *node, "Button assigned to move robot left");
    const std::string y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *node, "Button assigned to move robot right");
    const std::string z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *node, "Button assigned to move robot up");
    const std::string z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *node, "Button assigned to move robot down");
    // Additional parameters
    x_max = rosnu::declare_and_get_param<float>("x_max", 1.0f, *node, "The maximum output value along that axis of movement");
    y_max = rosnu::declare_and_get_param<float>("y_max", 1.0f, *node, "The maximum output value along that axis of movement");
    z_max = rosnu::declare_and_get_param<float>("z_max", 1.0f, *node, "The maximum output value along that axis of movement");
    alt_x_max = rosnu::declare_and_get_param<float>("alt_x_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_y_max = rosnu::declare_and_get_param<float>("alt_y_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_z_max = rosnu::declare_and_get_param<float>("alt_z_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    incr_mult = rosnu::declare_and_get_param<float>("incr_mult", 1.0f, *node, "The scale in which the movement speed is multiplied by along that axis of movement");
    x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *node, "Whether the input for this movement should be flipped");
    y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *node, "Whether the input for this movement should be flipped");
    z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *node, "Whether the input for this movement should be flipped");
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
    RCLCPP_INFO(rclcpp::get_logger("point_stamped_incr_joystick"), ("Currently using the " + device_name + " input device").c_str());
    // Creating the button map from the input device config file
    std::map<std::string, int> button_map;
    for (const auto& it : input_device["mapping"])
    {
        button_map[it.first.as<std::string>()] = it.second.as<int>();
    }
    
    // Creating MovementInputs from the retrieved input assignments parameters and created button mapping
    // Enabling Delta movement
    MovementInput enable_input = function_input(enable_assignment, button_map);
    // Reset Delta position
    MovementInput reset_input = function_input(reset_assignment, button_map);
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

    // Ensure upon start up, the delta starts in the center position
    command = zero_command();
    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            if(control_enabled(enable_input))
            {
                geometry_msgs::msg::PointStamped temp_command = command;

                alt_enabled(alt_input);

                temp_command = axes_reset(reset_input, temp_command);
                temp_command = x_axis_inc(x_inc_input, temp_command);
                temp_command = x_axis_dec(x_dec_input, temp_command);
                temp_command = y_axis_inc(y_inc_input, temp_command);
                temp_command = y_axis_dec(y_dec_input, temp_command);
                temp_command = z_axis_inc(z_inc_input, temp_command);
                temp_command = z_axis_dec(z_dec_input, temp_command);
                temp_command = flip_movement(temp_command);

                command = temp_command;
            }
            command.header.stamp = current_time;

            pntstmpd_pos_pub->publish(command);
        }
        rclcpp::spin_some(node);
    }
    return 0;
}

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("point_stamped_incr_joystick"), "Provided input is empty");
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
            RCLCPP_ERROR(rclcpp::get_logger("point_stamped_incr_joystick"), "Unable to determine input type");
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
    }
    else
    {
        curr_x_max = alt_x_max;
        curr_y_max = alt_y_max;
        curr_z_max = alt_z_max;
    }
}

static geometry_msgs::msg::PointStamped axes_reset(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return temp_command;
    }
    if (latest_joy_state.buttons.at(input.index))
    {
        return zero_command();
    }

    return temp_command;
}

static geometry_msgs::msg::PointStamped zero_command()
{
    geometry_msgs::msg::PointStamped new_command;

    new_command.point.x = 0.0f;
    new_command.point.y = 0.0f;
    new_command.point.z = 0.0f;

    return new_command;
}

static geometry_msgs::msg::PointStamped x_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.x = new_command.point.x + (curr_x_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.x = new_command.point.x + (curr_x_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.x = new_command.point.x + (curr_x_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.x) > curr_x_max)
    {
        new_command.point.x = curr_x_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped x_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.x = new_command.point.x + (curr_x_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.x = new_command.point.x - (curr_x_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.x = new_command.point.x - (curr_x_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.x) > curr_x_max)
    {
        new_command.point.x = curr_x_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped y_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.y = new_command.point.y - (curr_y_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.y = new_command.point.y - (curr_y_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.y = new_command.point.y - (curr_y_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.y) > curr_y_max)
    {
        new_command.point.y = curr_y_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped y_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.y = new_command.point.y - (curr_y_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.y = new_command.point.y + (curr_y_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.y = new_command.point.y + (curr_y_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.y) > curr_y_max)
    {
        new_command.point.y = curr_y_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped z_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.z = new_command.point.z + (curr_z_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.z = new_command.point.z + (curr_z_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.z = new_command.point.z + (curr_z_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.z) > curr_z_max)
    {
        new_command.point.z = curr_z_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped z_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;
    
    switch (input.type)
    {
        case InputType::None:
            break;
        case InputType::Axis:
            new_command.point.z = new_command.point.z + (curr_z_max/incr_denom) * incr_mult * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            new_command.point.z = new_command.point.z - (curr_z_max/incr_denom) * incr_mult * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            new_command.point.z = new_command.point.z - (curr_z_max/incr_denom) * incr_mult * latest_joy_state.buttons.at(input.index);
            break;
    }

    if (abs(new_command.point.z) > curr_z_max)
    {
        new_command.point.z = curr_z_max;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped flip_movement(geometry_msgs::msg::PointStamped temp_command)
{
    temp_command.point.x = temp_command.point.x * pow(-1, x_flip);
    temp_command.point.y = temp_command.point.y * pow(-1, y_flip);
    temp_command.point.z = temp_command.point.z * pow(-1, z_flip);

    return temp_command;
}

static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}