/// @file
/// @brief Publishes a series of PostStamped commands for a robot to receive based on inputs from a control device
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
///  `~/x_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/y_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/z_max (float) [default 1.0]`      - The maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///  `~/alt_x_max (float) [default 0.25]`      - The alternative maximum output value along that axis of movement
///
///  `~/x_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/y_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///  `~/z_flip (bool) [default false]`      - Whether the input for this movement should be flipped
///
///  `~/always_enable (bool) [default false]`      - Whether control input is always enabled (USE WITH CAUTION)
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

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

static geometry_msgs::msg::PointStamped command, old_command;
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
static geometry_msgs::msg::PointStamped zero_command();

/// @brief Based on the current/input position, returns coords with an increase in x
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped x_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords with an decrease in x
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped x_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords with an increase in y
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped y_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords with an decrease in y
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped y_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords with an increase in z
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped z_axis_inc(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Based on the current/input position, returns coords with an decrease in z
/// @param input - The controller input that will indicate whether the position changes
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped z_axis_dec(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Returns flipped positions depenending on parameters
/// @param temp_command - The message that will be overwritten with new positions for the robot
static geometry_msgs::msg::PointStamped flip_movement(geometry_msgs::msg::PointStamped temp_command);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("point_stamped_mirror_joystick");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    // Publisher
    auto pos_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("desired_position", 100); // puhlishing rate has to be 100
    
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
    // Additional parameters
    x_max = rosnu::declare_and_get_param<float>("x_max", 1.0f, *node, "The maximum output value along that axis of movement");
    y_max = rosnu::declare_and_get_param<float>("y_max", 1.0f, *node, "The maximum output value along that axis of movement");
    z_max = rosnu::declare_and_get_param<float>("z_max", 1.0f, *node, "The maximum output value along that axis of movement");
    alt_x_max = rosnu::declare_and_get_param<float>("alt_x_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_y_max = rosnu::declare_and_get_param<float>("alt_y_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    alt_z_max = rosnu::declare_and_get_param<float>("alt_z_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
    x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *node, "Whether the input for this movement should be flipped");
    y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *node, "Whether the input for this movement should be flipped");
    z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *node, "Whether the input for this movement should be flipped");
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
    RCLCPP_INFO(rclcpp::get_logger("point_stamped_mirror_joystick"), ("Currently using the " + device_name + " input device").c_str());
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

    // Ensure upon start up, the message starts in the center position
    command = zero_command();
    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            command = zero_command();

            if(control_enabled(enable_input))
            {
                geometry_msgs::msg::PointStamped temp_command = command;

                alt_enabled(alt_input);

                temp_command = z_axis_inc(z_inc_input, temp_command);
                temp_command = z_axis_dec(z_dec_input, temp_command);
                temp_command = x_axis_inc(x_inc_input, temp_command);
                temp_command = x_axis_dec(x_dec_input, temp_command);
                temp_command = y_axis_inc(y_inc_input, temp_command);
                temp_command = y_axis_dec(y_dec_input, temp_command);
                temp_command = flip_movement(temp_command);

                command = temp_command;
            }
        }

        command.header.stamp = current_time;
        pos_pub->publish(command);
        rclcpp::spin_some(node);
    }
    return 0;
}

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("point_stamped_mirror_joystick"), "Provided input is empty");
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
            RCLCPP_ERROR(rclcpp::get_logger("point_stamped_mirror_joystick"), "Unable to determine input type");
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

static geometry_msgs::msg::PointStamped zero_command()
{
    geometry_msgs::msg::PointStamped new_command;

    new_command.point.x = 0.0;
    new_command.point.y = 0.0;
    new_command.point.z = 0.0;

    return new_command;
}

static void alt_enabled(MovementInput input)
{
    if (input.type == InputType::None)
    {
        curr_x_max = x_max;
        curr_y_max = y_max;
        curr_z_max = z_max;
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

static geometry_msgs::msg::PointStamped x_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.x = curr_x_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.x = curr_x_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.x = curr_x_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped x_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.x = curr_x_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.x = -1 * curr_x_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.x = -1 * curr_x_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped y_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.y = curr_y_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.y = curr_y_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.y = curr_y_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped y_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.y = curr_y_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.y = -1 * curr_y_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.y = -1 * curr_y_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped z_axis_inc(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.z = curr_z_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.z = curr_z_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.z = curr_z_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped z_axis_dec(const MovementInput input, geometry_msgs::msg::PointStamped temp_command)
{
    geometry_msgs::msg::PointStamped new_command = temp_command;

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
                new_command.point.z = curr_z_max * axis_reading;
            }
            break;
        case InputType::Trigger:
            trigger_reading = (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            if (trigger_reading != 0.0f)
            {
                new_command.point.z = -1 * curr_z_max * trigger_reading;
            }
            break;
        case InputType::Button:
            button_reading = latest_joy_state.buttons.at(input.index);
            if (button_reading != 0.0f)
            {
                new_command.point.z = -1 * curr_z_max * button_reading;
            }
            break;
    }

    return new_command;
}

static geometry_msgs::msg::PointStamped flip_movement(geometry_msgs::msg::PointStamped temp_command)
{
    temp_command.point.x *= pow(-1, x_flip);
    temp_command.point.y *= pow(-1, y_flip);
    temp_command.point.z *= pow(-1, z_flip);

    return temp_command;
}


static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}