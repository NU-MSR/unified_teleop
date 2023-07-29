/// @file
/// @brief Publishes a series of series commands for the delta robot to move based on inputs from the gamepad
/// 
/// @section Publishers
///   delta/desired_position (geometry_msgs/PointStamped) - The desired position of the end effector
///
/// @section Subscribers
///   joy (sensor_msgs/Joy) - A message containing current state of the gamepad inputs
///
/// @section Parameters
///  `~/enable_delta (std::string) [default "r1_button"]`      - The name of the controller input that will control its respective function
///  `~/reset_delta (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_forward (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_backward (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_left (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_right (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_up (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///  `~/delta_down (std::string) [default "UNUSED"]`      - The name of the controller input that will control its respective function
///
///  `~/x_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/y_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement
///  `~/z_scale (float) [default 1.0]`      - The scale in which the movement speed is multiplied by along that axis of movement

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
static float x_scale;
static float y_scale;
static float z_scale;

static const float boundary_radius = 0.13f; // number based on same parameter in delta_position.cpp
static const float base_x_increment = 2 * std::pow(10, -5); // increment value chosen arbitrarily
static const float base_y_increment = 2 * std::pow(10, -5); // increment value chosen arbitrarily
static const float base_z_increment = 2 * std::pow(10, -5); // increment value chosen arbitrarily

enum class InputType
{
    Axis,
    Trigger,
    Button,
    None
};

static const int UNUSED_INDEX = -1;
static const InputType UNUSED_TYPE = InputType::None;

/// @brief a controller input
class MovementInput
{
    public:
        /// @brief each input has an index number that correlates with its position in the joy message array, and string that indicates the type of input it is (button, axis, trigger)
        int index;
        InputType type;

        // Constructors
        MovementInput() : index(UNUSED_INDEX), type(InputType::None) {}
        MovementInput(int index_no, InputType input_type) : index(index_no), type(input_type) {}
};

/// @brief Returns the type of the input based on its name (Axis, Trigger, Button, or None)
/// @param input_name - The name of the controller input
static InputType input_type(std::string input_name);

/// @brief Returns a MovementInput object using input assignment, along with the button mapping dictionary
/// @param input_name - The name of the controller input
static MovementInput function_input(std::string input_assignment, std::map<std::string, int> map);

/// @brief Handler for a joy message
/// @param joy_state - The states of the inputs of the controller
static void joy_callback(const sensor_msgs::msg::Joy & joy_state);

/// @brief Indicates whether delta movement has been enabled based on controller input
/// @param input - The controller input that will indicate whether the delta is enabled
static bool delta_enabled(MovementInput input);

/// @brief Resets the delta back to its original center position based on controller input
/// @param input - The controller input that will indicate whether the delta will reset its position
/// @param input_command - The message that will be overwritten with center position coordinates for the delta
static void delta_reset(MovementInput input, geometry_msgs::msg::PointStamped* input_command);

/// @brief Resets the delta back to its original center position
/// @param input_command - The message that will be overwritten with center position coordinates for the delta
static void reset_position(geometry_msgs::msg::PointStamped* input_command);

/// @brief Moves the delta's position forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_forward(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position backward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_backward(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position left
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_left(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position right
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_right(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position up
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_up(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_down(MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

int main(int argc, char * argv[])
{
    // ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("delta_joystick");
    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    // Subscriber
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

    // Publisher
    auto delta_pos_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("delta/desired_position", 100); // puhlishing rate has to be 100, otherwise delta displays incorrect behaviour
    
    //
    // Declaring and getting parameters
    //
    // Function -> Controller input assignments from control scheme parameters
    const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_delta", "UNUSED", *node, "Button assigned to enable delta movement");
    const std::string reset_assignment = rosnu::declare_and_get_param<std::string>("reset_delta", "UNUSED", *node, "Button assigned to reset delta position");
    const std::string forward_assignment = rosnu::declare_and_get_param<std::string>("delta_forward", "UNUSED", *node, "Button assigned to move delta forward");
    const std::string backward_assignment = rosnu::declare_and_get_param<std::string>("delta_backward", "UNUSED", *node, "Button assigned to move delta backward");
    const std::string left_assignment = rosnu::declare_and_get_param<std::string>("delta_left", "UNUSED", *node, "Button assigned to move delta left");
    const std::string right_assignment = rosnu::declare_and_get_param<std::string>("delta_right", "UNUSED", *node, "Button assigned to move delta right");
    const std::string up_assignment = rosnu::declare_and_get_param<std::string>("delta_up", "UNUSED", *node, "Button assigned to move delta up");
    const std::string down_assignment = rosnu::declare_and_get_param<std::string>("delta_down", "UNUSED", *node, "Button assigned to move delta down");
    // Creating a controller input -> associated joy message index number dictionary from the input device config file
    // YAML::Node input_device = YAML::LoadFile("$(find omnid_teleop)/config/test_config.yaml");
    const std::string input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device", "dualshock4_mapping", *node, "Chosen input device config file");
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("omnid_teleop");
    std::string full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
    YAML::Node input_device = YAML::LoadFile(full_path);

    const std::string device_name = input_device["name"].as<string>();
    RCLCPP_INFO(rclcpp::get_logger("delta_joystick"), ("Currently using the " + device_name + " input device").c_str());

    std::map<std::string, int> button_map;
    for (const auto& it : input_device["mapping"])
    {
        button_map[it.first.as<std::string>()] = it.second.as<int>();
    }
    
    // MovementInput from input assignments
    // Enabling Delta movement
    MovementInput enable_input = function_input(enable_assignment, button_map);
    // Reset Delta position
    MovementInput reset_input = function_input(reset_assignment, button_map);
    // Moving Delta forward
    MovementInput forward_input = function_input(forward_assignment, button_map);
    // Moving Delta backward
    MovementInput backward_input = function_input(backward_assignment, button_map);
    // Moving Delta left
    MovementInput left_input = function_input(left_assignment, button_map);
    // Moving Delta right
    MovementInput right_input = function_input(right_assignment, button_map);
    // Moving Delta up
    MovementInput up_input = function_input(up_assignment, button_map);
    // Moving Delta down
    MovementInput down_input = function_input(down_assignment, button_map);

    // Aditional parameters
    x_scale = rosnu::declare_and_get_param<float>("x_scale", 1.0f, *node, "The scale in which the movement speed is multiplied by along that axis of movement");
    y_scale = rosnu::declare_and_get_param<float>("y_scale", 1.0f, *node, "The scale in which the movement speed is multiplied by along that axis of movement");
    z_scale = rosnu::declare_and_get_param<float>("z_scale", 1.0f, *node, "The scale in which the movement speed is multiplied by along that axis of movement");

    // Ensure delta starts in correct position
    reset_position(&command);
    // Control loop
    while (rclcpp::ok())
    {
        rclcpp::Time current_time = rclcpp::Clock().now();

        if(fresh_joy_state)
        {
            if(delta_enabled(enable_input))
            {
                geometry_msgs::msg::PointStamped temp_command = command;

                delta_reset(reset_input, &temp_command);
                delta_forward(forward_input, &temp_command);
                delta_backward(backward_input, &temp_command);
                delta_left(left_input, &temp_command);
                delta_right(right_input, &temp_command);
                delta_up(up_input, &temp_command);
                delta_down(down_input, &temp_command);

                const float home_z_position = (PARAMETERS_WORK_Z_MIN_M + PARAMETERS_WORK_Z_MAX_M) / 2.0f;
                const float distance_from_home = sqrt(pow(temp_command.point.x, 2) + pow(temp_command.point.y, 2) + pow(temp_command.point.z - home_z_position, 2));

                if(distance_from_home < boundary_radius)
                {
                    command = temp_command;
                }
            }
            
            command.header.stamp = current_time;
            
            geometry_msgs::msg::PointStamped rounded_command = command;
            rounded_command.point.x = std::round(rounded_command.point.x * 1000)/1000;
            rounded_command.point.y = std::round(rounded_command.point.y * 1000)/1000;
            rounded_command.point.z = std::round(rounded_command.point.z * 1000)/1000;

            delta_pos_pub->publish(rounded_command);
        }
        rclcpp::spin_some(node);
    }
    return 0;
}

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("delta_joystick"), "Provided input is empty");
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
            RCLCPP_ERROR(rclcpp::get_logger("delta_joystick"), "Unable to determine input type");
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

static bool delta_enabled(const MovementInput input)
{
    return latest_joy_state.buttons.at(input.index);
}

static void delta_reset(const MovementInput input, geometry_msgs::msg::PointStamped* input_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (latest_joy_state.buttons.at(input.index))
    {
        reset_position(input_command);
    }
}

static void reset_position(geometry_msgs::msg::PointStamped* input_command)
{
    input_command->point.x = 0.0f;
    input_command->point.y = 0.0f;
    input_command->point.z = 0.355f;
    input_command->header.stamp = rclcpp::Clock().now();
}

static void delta_forward(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.x = temp_command->point.x + base_x_increment * x_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.x = temp_command->point.x + base_x_increment * x_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.x = temp_command->point.x + base_x_increment * x_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}

static void delta_backward(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.x = temp_command->point.x + base_x_increment * x_scale * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.x = temp_command->point.x - base_x_increment * x_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.x = temp_command->point.x - base_x_increment * x_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}

static void delta_left(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.y = temp_command->point.y - base_y_increment * y_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.y = temp_command->point.y - base_y_increment * y_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.y = temp_command->point.y - base_y_increment * y_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}

static void delta_right(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.y = temp_command->point.y - base_y_increment * y_scale * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.y = temp_command->point.y + base_y_increment * y_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.y = temp_command->point.y + base_y_increment * y_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}

static void delta_up(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.z = temp_command->point.z + base_z_increment * z_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.z = temp_command->point.z + base_z_increment * z_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.z = temp_command->point.z + base_z_increment * z_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}

static void delta_down(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    switch (input.type)
    {
        case InputType::None:
            return;
        case InputType::Axis:
            temp_command->point.z = temp_command->point.z + base_z_increment * z_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
            break;
        case InputType::Trigger:
            temp_command->point.z = temp_command->point.z - base_z_increment * z_scale * (0.5 - (latest_joy_state.axes.at(input.index)/2.0));
            break;
        case InputType::Button:
            temp_command->point.z = temp_command->point.z - base_z_increment * z_scale * latest_joy_state.buttons.at(input.index);
            break;
    }

    return;
}


static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}