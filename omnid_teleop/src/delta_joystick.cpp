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
#include <math.h>

using std::string;

static geometry_msgs::msg::PointStamped command;
static sensor_msgs::msg::Joy latest_joy_state;
static bool fresh_joy_state = false;
float x_scale;
float y_scale;
float z_scale;

static const float boundary_radius = 0.13f; // number based on same parameter in delta_position.cpp
static const float z_increment = 2 * std::pow(10, -5); // increment value chosen arbitrarily
static const float xy_increment = 2 * std::pow(10, -5); // increment value chosen arbitrarily

static const int UNUSED_INDEX = -1;
static const std::string UNUSED_TYPE = "";

/// @brief a controller input
class MovementInput
{
    public:
        /// @brief each input has an index number that correlates with its position in the joy message array, and string that indicates the type of input it is (button, axis, trigger)
        int index;
        std::string type;

        // Constructors
        MovementInput(int index_no, std::string input_type) : index(index_no), type(input_type) {}
        MovementInput() : index(UNUSED_INDEX), type(UNUSED_TYPE) {}
};

/// @brief Handler for a joy message
/// @param joy_state - The states of the inputs of the controller
static void joy_callback(const sensor_msgs::msg::Joy & joy_state);

/// @brief Indicates whether delta movement has been enabled based on controller input
/// @param input - The controller input that will indicate whether the delta is enabled
static bool delta_enabled(const MovementInput input);

/// @brief Resets the delta back to its original center position based on controller input
/// @param input - The controller input that will indicate whether the delta will reset its position
/// @param input_command - The message that will be overwritten with center position coordinates for the delta
static void delta_reset(const MovementInput input, geometry_msgs::msg::PointStamped* input_command);

/// @brief Resets the delta back to its original center position
/// @param input_command - The message that will be overwritten with center position coordinates for the delta
static void reset_position(geometry_msgs::msg::PointStamped* input_command);

/// @brief Moves the delta's position forward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_forward(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position backward
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_backward(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position left
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_left(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position right
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_right(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position up
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_up(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

/// @brief Moves the delta's position down
/// @param temp_command - The message that will be overwritten with new position coordinates for the delta
static void delta_down(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command);

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
    // Input assignments
    // Enabling Delta movement
    const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_delta", "r1_button", *node, "Button assigned to enable delta movement");
    MovementInput enable_input(rosnu::declare_and_get_param<int>(enable_assignment, *node, "Index to read to enable delta movement"), rosnu::declare_and_get_param<std::string>(enable_assignment + "_type", *node, "List to read to enable delta movement"));
    // Reset Delta position
    const std::string reset_assignment = rosnu::declare_and_get_param<std::string>("reset_delta", "UNUSED", *node, "Button assigned to reset delta position");
    MovementInput reset_input;
    if (reset_assignment != "UNUSED")
    {
        reset_input = MovementInput(rosnu::declare_and_get_param<int>(reset_assignment, *node, "Index to read to reset delta position"), rosnu::declare_and_get_param<std::string>(reset_assignment + "_type", *node, "List to read to reset delta position"));
    }
    // Moving Delta forward
    const std::string forward_assignment = rosnu::declare_and_get_param<std::string>("delta_forward", "UNUSED", *node, "Button assigned to move delta forward");
    MovementInput forward_input;
    if (forward_assignment != "UNUSED")
    {
        forward_input = MovementInput(rosnu::declare_and_get_param<int>(forward_assignment, *node, "Index to read to move delta forward"), rosnu::declare_and_get_param<std::string>(forward_assignment + "_type", *node, "List to read to move delta forward"));
    }
    // Moving Delta backward
    const std::string backward_assignment = rosnu::declare_and_get_param<std::string>("delta_backward", "UNUSED", *node, "Button assigned to move delta backward");
    MovementInput backward_input;
    if (backward_assignment != "UNUSED")
    {
        if (forward_assignment == backward_assignment)
        {
            backward_input = MovementInput(rosnu::get_param<int>(backward_assignment, *node), rosnu::get_param<std::string>(backward_assignment + "_type", *node));
        }
        else
        {
            backward_input = MovementInput(rosnu::declare_and_get_param<int>(backward_assignment, *node, "Index to read to move delta backward"), rosnu::declare_and_get_param<std::string>(backward_assignment + "_type", *node, "List to read to move delta backward"));
        }
    }
    // Moving Delta left
    const std::string left_assignment = rosnu::declare_and_get_param<std::string>("delta_left", "UNUSED", *node, "Button assigned to move delta left");
    MovementInput left_input;
    if (left_assignment != "UNUSED")
    {
        left_input = MovementInput(rosnu::declare_and_get_param<int>(left_assignment, *node, "Index to read to move delta left"), rosnu::declare_and_get_param<std::string>(left_assignment + "_type", *node, "List to read to move delta left"));
    }
    // Moving Delta right
    const std::string right_assignment = rosnu::declare_and_get_param<std::string>("delta_right", "UNUSED", *node, "Button assigned to move delta right");
    MovementInput right_input;
    if (right_assignment != "UNUSED")
    {
        if (left_assignment == right_assignment)
        {
            right_input = MovementInput(rosnu::get_param<int>(right_assignment, *node), rosnu::get_param<std::string>(right_assignment + "_type", *node));
        }
        else
        {
            right_input = MovementInput(rosnu::declare_and_get_param<int>(right_assignment, *node, "Index to read to move delta right"), rosnu::declare_and_get_param<std::string>(right_assignment + "_type", *node, "List to read to move delta right"));
        }
    }
    // Moving Delta up
    const std::string up_assignment = rosnu::declare_and_get_param<std::string>("delta_up", "UNUSED", *node, "Button assigned to move delta up");
    MovementInput up_input;
    if (up_assignment != "UNUSED")
    {
        up_input = MovementInput(rosnu::declare_and_get_param<int>(up_assignment, *node, "Index to read to move delta up"), rosnu::declare_and_get_param<std::string>(up_assignment + "_type", *node, "List to read to move delta up"));
    }
    // Moving Delta down
    const std::string down_assignment = rosnu::declare_and_get_param<std::string>("delta_down", "UNUSED", *node, "Button assigned to move delta down");
    MovementInput down_input;
    if (down_assignment != "UNUSED")
    {
        if (up_assignment == down_assignment)
        {
            down_input = MovementInput(rosnu::get_param<int>(down_assignment, *node), rosnu::get_param<std::string>(down_assignment + "_type", *node));
        }
        else
        {
            down_input = MovementInput(rosnu::declare_and_get_param<int>(down_assignment, *node, "Index to read to move delta down"), rosnu::declare_and_get_param<std::string>(down_assignment + "_type", *node, "List to read to move delta down"));
        }
    }
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

static bool delta_enabled(const MovementInput input)
{
    return (latest_joy_state.buttons.at(input.index) ? true : false);
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
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.x = temp_command->point.x + xy_increment * x_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.x = temp_command->point.x + xy_increment * x_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.x = temp_command->point.x + xy_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * x_scale;
    }
}

static void delta_backward(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.x = temp_command->point.x - xy_increment * x_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.x = temp_command->point.x + xy_increment * x_scale * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.x = temp_command->point.x - xy_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * x_scale;
    }
}

static void delta_left(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.y = temp_command->point.y - xy_increment * y_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.y = temp_command->point.y - xy_increment * y_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.y = temp_command->point.y - xy_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * y_scale;
    }
}

static void delta_right(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.y = temp_command->point.y + xy_increment * y_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.y = temp_command->point.y - xy_increment * y_scale * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.y = temp_command->point.y + xy_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * y_scale;
    }
}

static void delta_up(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.z = temp_command->point.z + z_increment * z_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.z = temp_command->point.z + z_increment * z_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.z = temp_command->point.z + z_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * z_scale;
    }
}

static void delta_down(const MovementInput input, geometry_msgs::msg::PointStamped* temp_command)
{
    if (input.type == UNUSED_TYPE)
    {
        return;
    }
    if (input.type == "button")
    {
        temp_command->point.z = temp_command->point.z - z_increment * z_scale * latest_joy_state.buttons.at(input.index);
    }
    else if (input.type == "axis")
    {
        temp_command->point.z = temp_command->point.z + z_increment * z_scale * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0);
    }
    else if (input.type == "trigger")
    {
        temp_command->point.z = temp_command->point.z - z_increment * (0.5 - (latest_joy_state.axes.at(input.index)/2)) * z_scale;
    }
}


static void joy_callback(const sensor_msgs::msg::Joy & joy_state)
{
    latest_joy_state = joy_state;
    fresh_joy_state = true;
}