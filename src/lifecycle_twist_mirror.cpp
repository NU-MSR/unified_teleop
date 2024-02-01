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
///  `~/lin_rate_chg_fac (float) [default 0.0]`     - Factor to the rate of change for the output's linear values
///  `~/ang_rate_chg_fac (float) [default 0.0]`     - Factor to the rate of change for the output's angular values
///  `~/x_offset (float) [default 0.0]`             - The offset for the message's zero value
///  `~/y_offset (float) [default 0.0]`             - The offset for the message's zero value
///  `~/z_offset (float) [default 0.0]`             - The offset for the message's zero value
///  `~/yaw_offset (float) [default 0.0]`           - The offset for the message's zero value
///  `~/pitch_offset (float) [default 0.0]`         - The offset for the message's zero value
///  `~/roll_offset (float) [default 0.0]`          - The offset for the message's zero value
///
///  `~/always_enable (bool) [default false]`      - Whether control input is always enabled (USE WITH CAUTION)
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

// For Lifecycle Nodes
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

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
static float x_offset;
static float y_offset;
static float z_offset;
static float yaw_offset;
static float pitch_offset;
static float roll_offset;
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

/// Additional parameter helper functions developed my NU MSR
namespace rosnu
{
  /// @brief declare a parameter without a default value. If the value is not set externally,
  /// an exception will be thrown when trying to get_param for this parameter.
  /// @tparam T - type of the parameter
  /// @param name - name of the parameter
  /// @param node - node for which the parameter is declared
  /// @param desc - (optional) the parameter description
  /// @throw 
  ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
  ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
  template<class T>
  void declare_param(const std::string & name, rclcpp::Node & node, const std::string & desc="")
  {
    // init descriptor object and fill in description
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = desc;

    // declare parameter without a default value
    node.declare_parameter<T>(name, descriptor);
  }

  /// @brief declare a parameter with a default value.
  /// @tparam T - type of the parameter
  /// @param name - name of the parameter
  /// @param def - the default parameter value
  /// @param node - node for which the parameter is declared
  /// @param desc - (optional) the parameter description
  /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
  template<class T>
  void declare_param(const std::string & name, const T & def, rclcpp::Node & node, const std::string & desc="")
  {
    // init descriptor object and fill in description
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = desc;
    
    // declare node with default value
    node.declare_parameter<T>(name, def, descriptor);
  }

  /// @brief get the value of a parameter.
  /// @tparam T - type of the parameter
  /// @param name - name of the parameter
  /// @param node - node for which the parameter was declared
  /// @return value of the parameter
  /// @throw rclcpp::exceptions::ParameterNotDeclaredException if the parameter has not been declared
  template<class T>
  T get_param(const std::string & name, rclcpp::Node & node)
  {
    return node.get_parameter(name).get_parameter_value().get<T>();
  }
  
  /// @brief declare a parameter without a default value and return the parameter value.
  /// @tparam T - type of the parameter
  /// @param name - name of the parameter
  /// @param node - node for which the parameter is declared
  /// @param desc - (optional) the parameter description
  /// @return value of the parameter
  /// @throw 
  ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
  ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
  template<class T>
  T declare_and_get_param(const std::string & name, rclcpp::Node & node, const std::string & desc="")
  {
    declare_param<T>(name, node, desc);
    return get_param<T>(name, node);
  }

  /// @brief declare a parameter with a default value and return the parameter value.
  /// @tparam T - type of the parameter
  /// @param name - name of the parameter
  /// @param def - the default parameter value
  /// @param node - node for which the parameter is declared
  /// @param desc - (optional) the parameter description
  /// @return value of the parameter
  /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
  template<class T>
  T declare_and_get_param(const std::string & name, const T & def, rclcpp::Node & node, const std::string & desc="")
  {
    declare_param<T>(name, def, node, desc);
    return get_param<T>(name, node);
  }
}

/// @brief Returns the type of the input based on its name
///        (Axis if it begins with an 'a', Trigger if it begins with a 't', Button if it begins with a 'b', and None if the string is empty)
/// @param input_name - The name of the controller input
static InputType input_type(std::string input_name);

/// @brief Returns a MovementInput object using its input assignment name and the button mapping dictionary
/// @param input_assignment - The name of the input device's input that is used for that function
/// @param map - The button mapping based on the input device config file
static MovementInput function_input(std::string input_assignment, std::map<std::string, int> map);

// /// @brief Handler for a joy message
// /// @param joy_state - The state of the inputs of the controller
// static void joy_callback(const sensor_msgs::msg::Joy & joy_state);

/// @brief Returns true if control inputs are enabled based on controller input
/// @param input - The controller input that will enable this function
static bool control_enabled(MovementInput input);

/// @brief Adjusts max values to set alternative values based on controller input
/// @param input - The controller input that will enable this function
static void alt_enabled(MovementInput input);

/// @brief Returns a Twist command that has zero for all of its fields
static geometry_msgs::msg::Twist zero_command();

/// @brief Returns a Twist command that has the offset for all of its fields
static geometry_msgs::msg::Twist offset_command();

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

/// @brief Returns a Twist message that reflects the difference between two given Twist messages
/// @param subtracted - The Twist message that will be subtracted
/// @param subtractor - The Twist message that will be subtracting from the subtracted
static geometry_msgs::msg::Twist subtract_twist(geometry_msgs::msg::Twist subtracted, geometry_msgs::msg::Twist subtractor);

/// @brief Returns a Twist message that normalizes the linear and angular components independently of each other to a given magnitude each
/// @param input_command - The Twist message that will be normalized
/// @param new_mag_linear - The desired magnitude for the linear component of resulting Twist message
/// @param new_mag_angular - The desired magnitude for the angular component of resulting Twist message
static geometry_msgs::msg::Twist normalize_twist(geometry_msgs::msg::Twist input_command, float new_mag_linear, float new_mag_angular);

/// @brief Returns a Twist message that reflects the sum between two given Twist messages
/// @param subtracted - First part of Twist addition
/// @param subtractor - Second part of the Twist addition
static geometry_msgs::msg::Twist add_twist(geometry_msgs::msg::Twist add1, geometry_msgs::msg::Twist add2);

/// @brief Returns a Twist message that with rounded values to a certain decimal point
/// @param input_command - The Twist message that will be rounded
static geometry_msgs::msg::Twist round_twist(geometry_msgs::msg::Twist input_command);

using namespace std::chrono_literals;
using std::placeholders::_1;

class LifecycleTwistMirrorNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        bool is_joy_freq;
        double pub_frequency;
        MovementInput enable_input;
        MovementInput reset_input;
        MovementInput alt_input;
        MovementInput x_inc_input;
        MovementInput x_dec_input;
        MovementInput y_inc_input;
        MovementInput y_dec_input;
        MovementInput z_inc_input;
        MovementInput z_dec_input;
        MovementInput yaw_inc_input;
        MovementInput yaw_dec_input;
        MovementInput pitch_inc_input;
        MovementInput pitch_dec_input;
        MovementInput roll_inc_input;
        MovementInput roll_dec_input;

        LifecycleTwistMirrorNode(bool intra_process_comms = false) : LifecycleNode("lifecycle_twist_mirror",
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            //
            // PARAMETERS
            //
            // Frequency of publisher
            declare_parameter("frequency", 95.);
            pub_frequency = get_parameter("frequency").as_double();
            // double pub_frequency = rosnu::declare_param<double>("frequency", 100.0f, *this, "Frequency of teleoperation output");
            // Function -> Controller input assignments from control scheme parameters
            declare_parameter("enable_control", "UNUSED");
            const std::string enable_assignment = get_parameter("enable_control").as_string();
            declare_parameter("reset_enable", "UNUSED");
            const std::string reset_assignment = get_parameter("reset_enable").as_string();
            declare_parameter("alt_enable", "UNUSED");
            const std::string alt_assignment = get_parameter("alt_enable").as_string();

            declare_parameter("x_axis_inc", "UNUSED");
            const std::string x_inc_assignment = get_parameter("x_axis_inc").as_string();
            declare_parameter("x_axis_dec", "UNUSED");
            const std::string x_dec_assignment = get_parameter("x_axis_dec").as_string();
            declare_parameter("y_axis_inc", "UNUSED");
            const std::string y_inc_assignment = get_parameter("y_axis_inc").as_string();
            declare_parameter("y_axis_dec", "UNUSED");
            const std::string y_dec_assignment = get_parameter("y_axis_dec").as_string();
            declare_parameter("z_axis_inc", "UNUSED");
            const std::string z_inc_assignment = get_parameter("z_axis_inc").as_string();
            declare_parameter("z_axis_dec", "UNUSED");
            const std::string z_dec_assignment = get_parameter("z_axis_dec").as_string();

            declare_parameter("yaw_inc", "UNUSED");
            const std::string yaw_inc_assignment = get_parameter("yaw_inc").as_string();
            declare_parameter("yaw_dec", "UNUSED");
            const std::string yaw_dec_assignment = get_parameter("yaw_dec").as_string();
            declare_parameter("pitch_inc", "UNUSED");
            const std::string pitch_inc_assignment = get_parameter("pitch_inc").as_string();
            declare_parameter("pitch_dec", "UNUSED");
            const std::string pitch_dec_assignment = get_parameter("pitch_dec").as_string();
            declare_parameter("roll_inc", "UNUSED");
            const std::string roll_inc_assignment = get_parameter("roll_inc").as_string();
            declare_parameter("roll_dec", "UNUSED");
            const std::string roll_dec_assignment = get_parameter("roll_dec").as_string();

            // Additional parameters
            declare_parameter("x_max", 1.);
            x_max = get_parameter("x_max").as_double();
            declare_parameter("y_max", 1.);
            y_max = get_parameter("y_max").as_double();
            declare_parameter("z_max", 1.);
            z_max = get_parameter("z_max").as_double();
            declare_parameter("alt_x_max", 0.25);
            alt_x_max = get_parameter("alt_x_max").as_double();
            declare_parameter("alt_y_max", 0.25);
            alt_y_max = get_parameter("alt_y_max").as_double();
            declare_parameter("alt_z_max", 0.25);
            alt_z_max = get_parameter("alt_z_max").as_double();
            declare_parameter("x_flip", false);
            x_flip = get_parameter("x_flip").as_bool();
            declare_parameter("y_flip", false);
            y_flip = get_parameter("y_flip").as_bool();
            declare_parameter("z_flip", false);
            z_flip = get_parameter("z_flip").as_bool();

            declare_parameter("yaw_max", 1.);
            yaw_max = get_parameter("yaw_max").as_double();
            declare_parameter("pitch_max", 1.);
            pitch_max = get_parameter("pitch_max").as_double();
            declare_parameter("roll_max", 1.);
            roll_max = get_parameter("roll_max").as_double();
            declare_parameter("alt_yaw_max", 0.25);
            alt_yaw_max = get_parameter("alt_yaw_max").as_double();
            declare_parameter("alt_pitch_max", 0.25);
            alt_pitch_max = get_parameter("alt_pitch_max").as_double();
            declare_parameter("alt_roll_max", 0.25);
            alt_roll_max = get_parameter("alt_roll_max").as_double();
            declare_parameter("yaw_flip", false);
            yaw_flip = get_parameter("yaw_flip").as_bool();
            declare_parameter("pitch_flip", false);
            pitch_flip = get_parameter("pitch_flip").as_bool();
            declare_parameter("roll_flip", false);
            roll_flip = get_parameter("roll_flip").as_bool();

            // Modifier parameters
            declare_parameter("lin_rate_chg_fac", 0.);
            lin_rate_chg_fac = get_parameter("lin_rate_chg_fac").as_double();
            // if (lin_rate_chg_fac == 0.0) // lin_rate_chg_fac cannot be 0.0
            // {
            //     lin_rate_chg_fac = 1.0;
            // }

            declare_parameter("ang_rate_chg_fac", 0.);
            ang_rate_chg_fac = get_parameter("ang_rate_chg_fac").as_double();
            // if (ang_rate_chg_fac == 0.0) // ang_rate_chg_fac cannot be 0.0
            // {
            //     ang_rate_chg_fac = 1.0;
            // }

            declare_parameter("x_offset", 0.);
            x_offset = get_parameter("x_offset").as_double();
            declare_parameter("y_offset", 0.);
            y_offset = get_parameter("y_offset").as_double();
            declare_parameter("z_offset", 0.);
            z_offset = get_parameter("z_offset").as_double();
            declare_parameter("yaw_offset", 0.);
            yaw_offset = get_parameter("yaw_offset").as_double();
            declare_parameter("pitch_offset", 0.);
            pitch_offset = get_parameter("pitch_offset").as_double();
            declare_parameter("roll_offset", 0.);
            roll_offset = get_parameter("roll_offset").as_double();


            declare_parameter("always_enable", false);
            always_enable = get_parameter("always_enable").as_bool();

            //
            // INTEGRATING INPUT & OUTPUT SCHEMES
            //
            // Getting the input device config from launch file parameters
            declare_parameter("input_device_config", "dualshock4_mapping");
            const std::string input_device_config_file = get_parameter("input_device_config").as_string();

            // const std::string input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device_config", "dualshock4_mapping", *this, "Chosen input device config file");
            // Creating a controller input -> associated joy message index number map from the input device config file
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("unified_teleop");
            std::string full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
            YAML::Node input_device = YAML::LoadFile(full_path);
            // Getting the input device name and printing it to the serial
            const std::string device_name = input_device["name"].as<string>();
            // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), ("Currently using the " + device_name + " input device").c_str());
            // Creating the button map from the input device config file
            std::map<std::string, int> button_map;
            for (const auto& it : input_device["mapping"])
            {
                button_map[it.first.as<std::string>()] = it.second.as<int>();
            }

            // Creating MovementInputs from the retrieved input assignments parameters and created button mapping
            enable_input = function_input(enable_assignment, button_map);
            alt_input = function_input(alt_assignment, button_map);
            x_inc_input = function_input(x_inc_assignment, button_map);
            x_dec_input = function_input(x_dec_assignment, button_map);
            y_inc_input = function_input(y_inc_assignment, button_map);
            y_dec_input = function_input(y_dec_assignment, button_map);
            z_inc_input = function_input(z_inc_assignment, button_map);
            z_dec_input = function_input(z_dec_assignment, button_map);
            yaw_inc_input = function_input(yaw_inc_assignment, button_map);
            yaw_dec_input = function_input(yaw_dec_assignment, button_map);
            pitch_inc_input = function_input(pitch_inc_assignment, button_map);
            pitch_dec_input = function_input(pitch_dec_assignment, button_map);
            roll_inc_input = function_input(roll_inc_assignment, button_map);
            roll_dec_input = function_input(roll_dec_assignment, button_map);
            
            //
            // INITIALIZING VARIABLES
            //
            command = zero_command();
            p_cmd = command;
            old_p_cmd = p_cmd;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            //
            // SUBSCRIBERS
            //
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&LifecycleTwistMirrorNode::joy_callback, this, _1));

            //
            // PUBLISHERS
            //
            cmdvel_pos_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
            
            //
            // TIMER CALLBACK
            //
            timer_ = this->create_wall_timer(1.0s/pub_frequency, std::bind(&LifecycleTwistMirrorNode::timer_callback, this));

            // Succesful transition
            RCLCPP_INFO(get_logger(), "on_configure() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state)
        {
            LifecycleNode::on_activate(state);
            RCUTILS_LOG_INFO_NAMED("lifecycle_twist_mirror", "on_activate() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & state)
        {   
            // Send zero command to mobile base so that it stops moving
            command = zero_command();
            cmdvel_pos_pub->publish(command);

            LifecycleNode::on_deactivate(state);
            RCUTILS_LOG_INFO_NAMED("lifecycle_twist_mirror", "on_deactivate() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            timer_.reset();
            cmdvel_pos_pub.reset();
            joy_sub.reset();

            RCUTILS_LOG_INFO_NAMED("lifecycle_twist_mirror", "on cleanup is called.");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state)
        {
            // In our shutdown phase, we release the shared pointers to the
            // timer and publisher. These entities are no longer available
            // and our node is "clean".
            timer_.reset();
            cmdvel_pos_pub.reset();
            joy_sub.reset();

            RCUTILS_LOG_INFO_NAMED(
            "lifecycle_twist_mirror",
            "on shutdown is called from state %s.",
            state.label().c_str());

            // We return a success and hence invoke the transition to the next
            // step: "finalized".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the current state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        void timer_callback()
        {
            if(cmdvel_pos_pub->is_activated())
            {
                // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), "TESTING 0");
                rclcpp::Time current_time = rclcpp::Clock().now();

                if(fresh_joy_state)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), "TESTING 1");
                    command = zero_command();

                    if(control_enabled(enable_input))
                    {
                        // Reading the raw Twist commands

                        // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), "TESTING 2");

                        alt_enabled(alt_input);
                        command = x_axis_inc(x_inc_input, command);
                        command = x_axis_dec(x_dec_input, command);
                        command = y_axis_inc(y_inc_input, command);
                        command = y_axis_dec(y_dec_input, command);
                        command = z_axis_inc(z_inc_input, command);
                        command = z_axis_dec(z_dec_input, command);

                        command = yaw_inc(yaw_inc_input, command);
                        command = yaw_dec(yaw_dec_input, command);
                        // RCLCPP_INFO(get_logger(), "COMMAND VAL: %f", static_cast<float>(command.angular.z));
                        command = pitch_inc(pitch_inc_input, command);
                        command = pitch_dec(pitch_dec_input, command);
                        command = roll_inc(roll_inc_input, command);
                        command = roll_dec(roll_dec_input, command);

                        command = flip_movement(command);
                        // RCLCPP_INFO(get_logger(), "COMMAND VAL: %f", static_cast<float>(command.linear.x));

                        // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), "TESTING 3");

                        // Implementing rate of change modifier
                        // Get the diff between curr and new
                        geometry_msgs::msg::Twist diff_twist = subtract_twist(command, p_cmd);
                        // Adjust the diff so that it's within the set rate_of_change
                        geometry_msgs::msg::Twist adjusted_diff = normalize_twist(diff_twist, rate_of_change * lin_rate_chg_fac, rate_of_change * ang_rate_chg_fac);
                        // Increment it on the new processed command
                        p_cmd = add_twist(p_cmd, adjusted_diff);
                    }
                    else
                    {
                        p_cmd = command;
                    }

                    old_p_cmd = p_cmd;

                    // Adjust command so that it is offset as desired
                    geometry_msgs::msg::Twist offset_cmd = add_twist(p_cmd, offset_command());
                    // Round the values of the message so that it does not sporadically change
                    offset_cmd = round_twist(offset_cmd);

                    cmdvel_pos_pub->publish(offset_cmd);
                }
            }
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_state) const
        {
            latest_joy_state = *joy_state;
            fresh_joy_state = true;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pos_pub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

};

int main(int argc, char * argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<LifecycleTwistMirrorNode>());
    // rclcpp::shutdown();
    // return 0;

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleTwistMirrorNode> lc_node = std::make_shared<LifecycleTwistMirrorNode>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

// int main(int argc, char * argv[])
// {
//     // ROS
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("twist_mirror");
//     rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

//     // Subscriber
//     auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);

//     // Publisher
//     auto cmdvel_pos_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100); // puhlishing rate has to be 100
    
//     //
//     // Declaring and getting parameters
//     //
//     // Function -> Controller input assignments from control scheme parameters
//     const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *node, "Button assigned to enable control inputs");
//     const std::string alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *node, "Button assigned to activate alternative max values");
//     const std::string x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *node, "Button assigned to increase the x-axis value of the robot");
//     const std::string x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *node, "Button assigned to decrease the x-axis value of the robot");
//     const std::string y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *node, "Button assigned to increase the y-axis value of the robot");
//     const std::string y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *node, "Button assigned to decrease the y-axis value of the robot");
//     const std::string z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *node, "Button assigned to increase the z-axis value of the robot");
//     const std::string z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *node, "Button assigned to decrease the z-axis value of the robot");
//     const std::string yaw_inc_assignment = rosnu::declare_and_get_param<std::string>("yaw_inc", "UNUSED", *node, "Button assigned to increase the yaw value of the robot");
//     const std::string yaw_dec_assignment = rosnu::declare_and_get_param<std::string>("yaw_dec", "UNUSED", *node, "Button assigned to decrease the yaw value of the robot");
//     const std::string pitch_inc_assignment = rosnu::declare_and_get_param<std::string>("pitch_inc", "UNUSED", *node, "Button assigned to increase the pitch value of the robot");
//     const std::string pitch_dec_assignment = rosnu::declare_and_get_param<std::string>("pitch_dec", "UNUSED", *node, "Button assigned to decrease the pitch value of the robot");
//     const std::string roll_inc_assignment = rosnu::declare_and_get_param<std::string>("roll_inc", "UNUSED", *node, "Button assigned to increase the roll value of the robot");
//     const std::string roll_dec_assignment = rosnu::declare_and_get_param<std::string>("roll_dec", "UNUSED", *node, "Button assigned to decrease the roll value of the robot");
//     // Additional parameters
//     x_max = rosnu::declare_and_get_param<float>("x_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     y_max = rosnu::declare_and_get_param<float>("y_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     z_max = rosnu::declare_and_get_param<float>("z_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     yaw_max = rosnu::declare_and_get_param<float>("yaw_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     pitch_max = rosnu::declare_and_get_param<float>("pitch_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     roll_max = rosnu::declare_and_get_param<float>("roll_max", 1.0f, *node, "The maximum output value along that axis of movement");
//     alt_x_max = rosnu::declare_and_get_param<float>("alt_x_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     alt_y_max = rosnu::declare_and_get_param<float>("alt_y_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     alt_z_max = rosnu::declare_and_get_param<float>("alt_z_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     alt_yaw_max = rosnu::declare_and_get_param<float>("alt_yaw_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     alt_pitch_max = rosnu::declare_and_get_param<float>("alt_pitch_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     alt_roll_max = rosnu::declare_and_get_param<float>("alt_roll_max", 0.25f, *node, "The alternative maximum output value along that axis of movement");
//     x_flip = rosnu::declare_and_get_param<bool>("x_flip", false, *node, "Whether the input for this movement should be flipped");
//     y_flip = rosnu::declare_and_get_param<bool>("y_flip", false, *node, "Whether the input for this movement should be flipped");
//     z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *node, "Whether the input for this movement should be flipped");
//     yaw_flip = rosnu::declare_and_get_param<bool>("yaw_flip", false, *node, "Whether the input for this movement should be flipped");
//     pitch_flip = rosnu::declare_and_get_param<bool>("pitch_flip", false, *node, "Whether the input for this movement should be flipped");
//     roll_flip = rosnu::declare_and_get_param<bool>("roll_flip", false, *node, "Whether the input for this movement should be flipped");
//     // Modifier parameters
//     lin_rate_chg_fac = rosnu::declare_and_get_param<float>("lin_rate_chg_fac", 0.0f, *node, "Factor to the rate of change for the output's linear values");
//     ang_rate_chg_fac = rosnu::declare_and_get_param<float>("ang_rate_chg_fac", 0.0f, *node, "Factor to the rate of change for the output's angular values");
//     x_offset = rosnu::declare_and_get_param<float>("x_offset", 0.0f, *node, "The offset for the message's zero value");
//     y_offset = rosnu::declare_and_get_param<float>("y_offset", 0.0f, *node, "The offset for the message's zero value");
//     z_offset = rosnu::declare_and_get_param<float>("z_offset", 0.0f, *node, "The offset for the message's zero value");
//     yaw_offset = rosnu::declare_and_get_param<float>("yaw_offset", 0.0f, *node, "The offset for the message's zero value");
//     pitch_offset = rosnu::declare_and_get_param<float>("pitch_offset", 0.0f, *node, "The offset for the message's zero value");
//     roll_offset = rosnu::declare_and_get_param<float>("roll_offset", 0.0f, *node, "The offset for the message's zero value");
//     // Whether control input is ALWAYS enabled
//     always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *node, "Whether control input is always enabled (USE WITH CAUTION)");
//     // Getting the input device config from launch file parameters
//     const std::string input_device_config_file = rosnu::declare_and_get_param<std::string>("input_device_config", "dualshock4_mapping", *node, "Chosen input device config file");
    
//     // Creating a controller input -> associated joy message index number map from the input device config file
//     std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("unified_teleop");
//     std::string full_path = pkg_share_dir + "/config/" + input_device_config_file + ".yaml";
//     YAML::Node input_device = YAML::LoadFile(full_path);
//     // Getting the input device name and printing it to the serial
//     const std::string device_name = input_device["name"].as<string>();
//     RCLCPP_INFO(rclcpp::get_logger("twist_mirror"), ("Currently using the " + device_name + " input device").c_str());
//     // Creating the button map from the input device config file
//     std::map<std::string, int> button_map;
//     for (const auto& it : input_device["mapping"])
//     {
//         button_map[it.first.as<std::string>()] = it.second.as<int>();
//     }
    
//     // Creating MovementInputs from the retrieved input assignments parameters and created button mapping
//     MovementInput enable_input = function_input(enable_assignment, button_map);
//     MovementInput alt_input = function_input(alt_assignment, button_map);
//     MovementInput x_inc_input = function_input(x_inc_assignment, button_map);
//     MovementInput x_dec_input = function_input(x_dec_assignment, button_map);
//     MovementInput y_inc_input = function_input(y_inc_assignment, button_map);
//     MovementInput y_dec_input = function_input(y_dec_assignment, button_map);
//     MovementInput z_inc_input = function_input(z_inc_assignment, button_map);
//     MovementInput z_dec_input = function_input(z_dec_assignment, button_map);
//     MovementInput yaw_inc_input = function_input(yaw_inc_assignment, button_map);
//     MovementInput yaw_dec_input = function_input(yaw_dec_assignment, button_map);
//     MovementInput pitch_inc_input = function_input(pitch_inc_assignment, button_map);
//     MovementInput pitch_dec_input = function_input(pitch_dec_assignment, button_map);
//     MovementInput roll_inc_input = function_input(roll_inc_assignment, button_map);
//     MovementInput roll_dec_input = function_input(roll_dec_assignment, button_map);

//     command = zero_command();
//     p_cmd = command;
//     old_p_cmd = p_cmd;
    
//     // Control loop
//     while (rclcpp::ok())
//     {
//         rclcpp::Time current_time = rclcpp::Clock().now();

//         if(fresh_joy_state)
//         {
//             command = zero_command();

//             if(control_enabled(enable_input))
//             {
//                 // Reading the raw Twist commands

//                 alt_enabled(alt_input);
//                 command = x_axis_inc(x_inc_input, command);
//                 command = x_axis_dec(x_dec_input, command);
//                 command = y_axis_inc(y_inc_input, command);
//                 command = y_axis_dec(y_dec_input, command);
//                 command = z_axis_inc(z_inc_input, command);
//                 command = z_axis_dec(z_dec_input, command);

//                 command = yaw_inc(yaw_inc_input, command);
//                 command = yaw_dec(yaw_dec_input, command);
//                 command = pitch_inc(pitch_inc_input, command);
//                 command = pitch_dec(pitch_dec_input, command);
//                 command = roll_inc(roll_inc_input, command);
//                 command = roll_dec(roll_dec_input, command);

//                 command = flip_movement(command);

//                 // Implementing rate of change modifier
//                 // Get the diff between curr and new
//                 geometry_msgs::msg::Twist diff_twist = subtract_twist(command, p_cmd);
//                 // Adjust the diff so that it's within the set rate_of_change
//                 geometry_msgs::msg::Twist adjusted_diff = normalize_twist(diff_twist, rate_of_change * lin_rate_chg_fac, rate_of_change * ang_rate_chg_fac);
//                 // Increment it on the new processed command
//                 p_cmd = add_twist(p_cmd, adjusted_diff);
//             }
//             else
//             {
//                 p_cmd = command;
//             }

//             old_p_cmd = p_cmd;

//             // Adjust command so that it is offset as desired
//             geometry_msgs::msg::Twist offset_cmd = add_twist(p_cmd, offset_command());
//             // Round the values of the message so that it does not sporadically change
//             offset_cmd = round_twist(offset_cmd);

//             cmdvel_pos_pub->publish(offset_cmd);
//         }
//         rclcpp::spin_some(node);
//     }
//     return 0;
// }

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("twist_mirror"), "Provided input is empty");
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
            RCLCPP_ERROR(rclcpp::get_logger("twist_mirror"), "Unable to determine input type");
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

    new_command.linear.x = 0.0;
    new_command.linear.y = 0.0;
    new_command.linear.z = 0.0;
    new_command.angular.x = 0.0;
    new_command.angular.y = 0.0;
    new_command.angular.z = 0.0;

    return new_command;
}

static geometry_msgs::msg::Twist offset_command()
{
    geometry_msgs::msg::Twist new_command;

    new_command.linear.x = x_offset;
    new_command.linear.y = y_offset;
    new_command.linear.z = z_offset;
    new_command.angular.x = yaw_offset;
    new_command.angular.y = pitch_offset;
    new_command.angular.z = roll_offset;

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
                // RCLCPP_INFO(rclcpp::get_logger("lifecycle_twist_mirror"), "TESTING 4");
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
            {RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TESTING CONSTRUCTOR");
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

static geometry_msgs::msg::Twist round_twist(geometry_msgs::msg::Twist input_command)
{
    geometry_msgs::msg::Twist new_command;
    int precision = 3;
    new_command.linear.x = round(input_command.linear.x * std::pow(10, precision))/std::pow(10, precision);
    new_command.linear.y = round(input_command.linear.y * std::pow(10, precision))/std::pow(10, precision);
    new_command.linear.z = round(input_command.linear.z * std::pow(10, precision))/std::pow(10, precision);
    new_command.angular.x = round(input_command.angular.x * std::pow(10, precision))/std::pow(10, precision);
    new_command.angular.y = round(input_command.angular.y * std::pow(10, precision))/std::pow(10, precision);
    new_command.angular.z = round(input_command.angular.z * std::pow(10, precision))/std::pow(10, precision);
    return new_command;
}