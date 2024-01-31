/// @file
/// @brief Publishes a series of PostStamped commands for a robot to receive based on inputs from a control device, with the values incrementing as the inputs are pressed
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
///  `~/boundary_radius (float) [default 0.0]`      - Radius of the spherical space around the zero position that the robot can move in
///  `~/lin_rate_chg_fac (float) [default 1.0]`     - Factor to the rate of change for the output's values
///  `~/x_offset (float) [default 0.0]`             - The offset for the message's zero value
///  `~/y_offset (float) [default 0.0]`             - The offset for the message's zero value
///  `~/z_offset (float) [default 0.0]`             - The offset for the message's zero value
///
///  `~/always_enable (bool) [default false]`      - Whether control input is always enabled (USE WITH CAUTION)
///
///  `~/input_device_config_file (std::string) [default "dualshock4_mapping"]`      - Chosen input device config file

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <string>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <functional>
#include <memory>

// Additional libraries for Lifecycle nodes
#include <thread>
#include <utility>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using std::string;

static geometry_msgs::msg::PointStamped command;
static sensor_msgs::msg::Joy latest_joy_state;
static bool fresh_joy_state;
static bool always_enable = false;
static const float rate_of_change_denom = 1 * std::pow(10, 5); // USER CAN ADJUST, KEEP IT EXTREMELY SMALL

float curr_x_max = 1.0;
float curr_y_max = 1.0;
float curr_z_max = 1.0;
static float x_max;
static float y_max;
static float z_max;
static float alt_x_max;
static float alt_y_max;
static float alt_z_max;
static float boundary_radius;
float lin_rate_chg_fac;
static float x_offset;
static float y_offset;
static float z_offset;
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

// /// Additional parameter helper functions developed my NU MSR
// namespace rosnu
// {
//   /// @brief declare a parameter without a default value. If the value is not set externally,
//   /// an exception will be thrown when trying to get_param for this parameter.
//   /// @tparam T - type of the parameter
//   /// @param name - name of the parameter
//   /// @param node - node for which the parameter is declared
//   /// @param desc - (optional) the parameter description
//   /// @throw 
//   ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
//   ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
//   template<class T>
//   void declare_param(const std::string & name, rclcpp::Node & node, const std::string & desc="")
//   {
//     // init descriptor object and fill in description
//     auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
//     descriptor.description = desc;

//     // declare parameter without a default value
//     node.declare_parameter<T>(name, descriptor);
//   }

//   /// @brief declare a parameter with a default value.
//   /// @tparam T - type of the parameter
//   /// @param name - name of the parameter
//   /// @param def - the default parameter value
//   /// @param node - node for which the parameter is declared
//   /// @param desc - (optional) the parameter description
//   /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
//   template<class T>
//   void declare_param(const std::string & name, const T & def, rclcpp::Node & node, const std::string & desc="")
//   {
//     // init descriptor object and fill in description
//     auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
//     descriptor.description = desc;
    
//     // declare node with default value
//     node.declare_parameter<T>(name, def, descriptor);
//   }

//   /// @brief get the value of a parameter.
//   /// @tparam T - type of the parameter
//   /// @param name - name of the parameter
//   /// @param node - node for which the parameter was declared
//   /// @return value of the parameter
//   /// @throw rclcpp::exceptions::ParameterNotDeclaredException if the parameter has not been declared
//   template<class T>
//   T get_param(const std::string & name, rclcpp::Node & node)
//   {
//     return node.get_parameter(name).get_parameter_value().get<T>();
//   }
  
//   /// @brief declare a parameter without a default value and return the parameter value.
//   /// @tparam T - type of the parameter
//   /// @param name - name of the parameter
//   /// @param node - node for which the parameter is declared
//   /// @param desc - (optional) the parameter description
//   /// @return value of the parameter
//   /// @throw 
//   ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
//   ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
//   template<class T>
//   T declare_and_get_param(const std::string & name, rclcpp::Node & node, const std::string & desc="")
//   {
//     declare_param<T>(name, node, desc);
//     return get_param<T>(name, node);
//   }

//   /// @brief declare a parameter with a default value and return the parameter value.
//   /// @tparam T - type of the parameter
//   /// @param name - name of the parameter
//   /// @param def - the default parameter value
//   /// @param node - node for which the parameter is declared
//   /// @param desc - (optional) the parameter description
//   /// @return value of the parameter
//   /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
//   template<class T>
//   T declare_and_get_param(const std::string & name, const T & def, rclcpp::Node & node, const std::string & desc="")
//   {
//     declare_param<T>(name, def, node, desc);
//     return get_param<T>(name, node);
//   }
// }

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

/// @brief Based on the input, resets the axes values of the PointStamped message
/// @param input - The controller input that will indicate whether the position resets
/// @param temp_command - The message that will be overwritten with new position coordinates for the robot
static geometry_msgs::msg::PointStamped axes_reset(MovementInput input, geometry_msgs::msg::PointStamped temp_command);

/// @brief Returns a PointStamped command that has zero for all of its fields
static geometry_msgs::msg::PointStamped zero_command();

/// @brief Returns a PointStamped command that has the offset for all of its fields
static geometry_msgs::msg::PointStamped offset_command();

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

/// @brief Returns a PointStamped message that normalizes the values to a given magnitude
/// @param input_command - The PointStamped message that will be normalized
/// @param new_mag - The desired magnitude for the resulting PointStamped message
static geometry_msgs::msg::PointStamped normalize_pntstmp(geometry_msgs::msg::PointStamped input_command, float new_mag);

/// @brief Returns a PointStamped message that reflects the sum between two given PointStamped messages
/// @param subtracted - First part of PointStamped addition
/// @param subtractor - Second part of the PointStamped addition
static geometry_msgs::msg::PointStamped add_pntstmp(geometry_msgs::msg::PointStamped add1, geometry_msgs::msg::PointStamped add2);

using namespace std::chrono_literals;
using std::placeholders::_1;

class LifecyclePointStampedIncrNode : public rclcpp_lifecycle::LifecycleNode
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

        LifecyclePointStampedIncrNode(bool intra_process_comms = false) : LifecycleNode("lifecycle_point_stamped_incr",
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TESTING CONSTRUCTOR");
            
            //
            // PARAMETERS
            //
            // Frequency of publisher
            declare_parameter("frequency", 0.);
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

            // const std::string enable_assignment = rosnu::declare_and_get_param<std::string>("enable_control", "UNUSED", *this, "Button assigned to enable control inputs");
            // const std::string reset_assignment = rosnu::declare_and_get_param<std::string>("reset_enable", "UNUSED", *this, "Button assigned to reset robot position");
            // const std::string alt_assignment = rosnu::declare_and_get_param<std::string>("alt_enable", "UNUSED", *this, "Button assigned to activate alternative max values");
            // const std::string x_inc_assignment = rosnu::declare_and_get_param<std::string>("x_axis_inc", "UNUSED", *this, "Button assigned to increase the x-axis value of the robot");
            // const std::string x_dec_assignment = rosnu::declare_and_get_param<std::string>("x_axis_dec", "UNUSED", *this, "Button assigned to decrease the x-axis value of the robot");
            // const std::string y_inc_assignment = rosnu::declare_and_get_param<std::string>("y_axis_inc", "UNUSED", *this, "Button assigned to increase the y-axis value of the robot");
            // const std::string y_dec_assignment = rosnu::declare_and_get_param<std::string>("y_axis_dec", "UNUSED", *this, "Button assigned to decrease the y-axis value of the robot");
            // const std::string z_inc_assignment = rosnu::declare_and_get_param<std::string>("z_axis_inc", "UNUSED", *this, "Button assigned to increase the z-axis value of the robot");
            // const std::string z_dec_assignment = rosnu::declare_and_get_param<std::string>("z_axis_dec", "UNUSED", *this, "Button assigned to decrease the z-axis value of the robot");
            
            // Additional parameters
            declare_parameter("x_max", 0.);
            x_max = get_parameter("x_max").as_double();
            declare_parameter("y_max", 0.);
            y_max = get_parameter("y_max").as_double();
            declare_parameter("z_max", 0.);
            z_max = get_parameter("z_max").as_double();
            declare_parameter("alt_x_max", 0.);
            alt_x_max = get_parameter("alt_x_max").as_double();
            declare_parameter("alt_y_max", 0.);
            alt_y_max = get_parameter("alt_y_max").as_double();
            declare_parameter("alt_z_max", 0.);
            alt_z_max = get_parameter("alt_z_max").as_double();
            declare_parameter("x_flip", 0.);
            x_flip = get_parameter("x_flip").as_double();
            declare_parameter("y_flip", 0.);
            y_flip = get_parameter("y_flip").as_double();
            declare_parameter("z_flip", 0.);
            z_flip = get_parameter("z_flip").as_double();

            // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TESTING PARAMETERS");

            // x_max = rosnu::declare_and_get_param<float>("x_max", 1.0f, *this, "The maximum output value along that axis of movement");
            // y_max = rosnu::declare_and_get_param<float>("y_max", 1.0f, *this, "The maximum output value along that axis of movement");
            // z_max = rosnu::declare_and_get_param<float>("z_max", 1.0f, *this, "The maximum output value along that axis of movement");
            // alt_x_max = rosnu::declare_and_get_param<float>("alt_x_max", 0.25f, *this, "The alternative maximum output value along that axis of LifecyclePointStampedIncrNode
            // z_flip = rosnu::declare_and_get_param<bool>("z_flip", false, *this, "Whether the input for this movement should be flipped");

            // Modifier parameters
            declare_parameter("boundary_radius", 0.);
            boundary_radius = get_parameter("boundary_radius").as_double();
            declare_parameter("lin_rate_chg_fac", 0.);
            lin_rate_chg_fac = get_parameter("lin_rate_chg_fac").as_double();

            // boundary_radius = rosnu::declare_and_get_param<float>("boundary_radius", 0.0f, *this, "Radius of the spherical space around the zero position that the robot can move in");
            // lin_rate_chg_fac = rosnu::declare_and_get_param<float>("lin_rate_chg_fac", 1.0f, *this, "The scale in which the movement speed is multiplied by along that axis of movement");
            
            if (lin_rate_chg_fac == 0.0) // lin_rate_chg_fac cannot be 0.0
            {
                lin_rate_chg_fac = 1.0;
            }

            declare_parameter("x_offset", 0.);
            x_offset = get_parameter("x_offset").as_double();
            declare_parameter("y_offset", 0.);
            y_offset = get_parameter("y_offset").as_double();
            declare_parameter("z_offset", 0.);
            z_offset = get_parameter("z_offset").as_double();

            // x_offset = rosnu::declare_and_get_param<float>("x_offset", 0.0f, *this, "The offset for the message's zero value");
            // y_offset = rosnu::declare_and_get_param<float>("y_offset", 0.0f, *this, "The offset for the message's zero value");
            // z_offset = rosnu::declare_and_get_param<float>("z_offset", 0.0f, *this, "The offset for the message's zero value");
            // Whether control input is ALWAYS enabled
            declare_parameter("always_enable", false);
            always_enable = get_parameter("always_enable").as_bool();

            // always_enable = rosnu::declare_and_get_param<bool>("always_enable", false, *this, "Whether control input is always enabled (USE WITH CAUTION)");

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
            // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), ("Currently using the " + device_name + " input device").c_str());
            // Creating the button map from the input device config file
            std::map<std::string, int> button_map;
            for (const auto& it : input_device["mapping"])
            {
                button_map[it.first.as<std::string>()] = it.second.as<int>();
            }
            // Creating MovementInputs from the retrieved input assignments parameters and created button mapping
            enable_input = function_input(enable_assignment, button_map);
            reset_input = function_input(reset_assignment, button_map);
            alt_input = function_input(alt_assignment, button_map);
            x_inc_input = function_input(x_inc_assignment, button_map);
            x_dec_input = function_input(x_dec_assignment, button_map);
            y_inc_input = function_input(y_inc_assignment, button_map);
            y_dec_input = function_input(y_dec_assignment, button_map);
            z_inc_input = function_input(z_inc_assignment, button_map);
            z_dec_input = function_input(z_dec_assignment, button_map);

            //
            // INITIALIZING VARIABLES
            //
            // Ensure upon start up, the robot starts in the center position
            command = zero_command();
            // If frequency set to 0, then only publishes messages when a new joy message is received
            // but timer will still have a frequency of 100.0
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

        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            //
            // SUBSCRIBERS
            //
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&LifecyclePointStampedIncrNode::joy_callback, this, _1));

            //
            // PUBLISHERS
            //
            pntstmpd_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("desired_position", 100);
            
            //
            // TIMER CALLBACK
            //
            timer_ = this->create_wall_timer(1.0s/pub_frequency, std::bind(&LifecyclePointStampedIncrNode::timer_callback, this));

            // Succesful transition
            RCLCPP_INFO(get_logger(), "on_configure() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & state)
        {
            LifecycleNode::on_activate(state);
            RCUTILS_LOG_INFO_NAMED("lifecycle_point_stamped_incr", "on_activate() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            timer_.reset();
            pntstmpd_pub.reset();
            joy_sub.reset();

            RCUTILS_LOG_INFO_NAMED("lifecycle_point_stamped_incr", "on cleanup is called.");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & state)
        {
            // In our shutdown phase, we release the shared pointers to the
            // timer and publisher. These entities are no longer available
            // and our node is "clean".
            timer_.reset();
            pntstmpd_pub.reset();
            joy_sub.reset();

            RCUTILS_LOG_INFO_NAMED(
            "lifecycle_point_stamped_incr",
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
            if(pntstmpd_pub->is_activated())
            {
                // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TESTING");
                rclcpp::Time current_time = rclcpp::Clock().now();

                if (fresh_joy_state == true)
                {
                    // If frequency set to 0, then only publishes messages when a new joy message is received
                    // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TEST 0");
                    if (is_joy_freq)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TEST 1");
                        fresh_joy_state = false;
                    }

                    if(control_enabled(enable_input))
                    {
                        // Reading the raw PointStamped commands

                        alt_enabled(alt_input);

                        // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TEST 2");

                        command = x_axis_inc(x_inc_input, command);
                        command = x_axis_dec(x_dec_input, command);
                        command = y_axis_inc(y_inc_input, command);
                        command = y_axis_dec(y_dec_input, command);
                        command = z_axis_inc(z_inc_input, command);
                        command = z_axis_dec(z_dec_input, command);
                        command = axes_reset(reset_input, command);

                        // Implementing spherical positional boundary modifier
                        // Make sure the robot's position is constrained to the desired spherical space
                        if (boundary_radius != 0.0)
                        {
                            // Find the robot's desired distance from home sqrd
                            float distance_from_home_sqrd = pow(command.point.x, 2) + pow(command.point.y, 2) + pow(command.point.z, 2);
                            // If the distance is larger than the desired boundary radius, normalize the position's magnitude so that it's within allowed space
                            if (distance_from_home_sqrd > pow(boundary_radius, 2))
                            {
                                command = normalize_pntstmp(command, boundary_radius);
                            }
                        }
                    }

                    // Adjust command so that it is offset as desired
                    geometry_msgs::msg::PointStamped offset_cmd = add_pntstmp(command, offset_command());

                    offset_cmd.header.stamp = current_time;
                    pntstmpd_pub->publish(offset_cmd);
                }
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr pntstmpd_pub;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_state) const
        {
            // RCLCPP_INFO(rclcpp::get_logger("lifecycle_point_stamped_incr"), "TEST JOY");
            latest_joy_state = *joy_state;
            fresh_joy_state = true;
        }
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
};

int main(int argc, char * argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<LifecyclePointStampedIncrNode>());
    // rclcpp::shutdown();
    // return 0;

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecyclePointStampedIncrNode> lc_node = std::make_shared<LifecyclePointStampedIncrNode>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

static InputType input_type(const std::string input_name)
{
    if (input_name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("lifecycle_point_stamped_incr"), "Provided input is empty");
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
            RCLCPP_ERROR(rclcpp::get_logger("lifecycle_point_stamped_incr"), "Unable to determine input type");
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

static geometry_msgs::msg::PointStamped offset_command()
{
    geometry_msgs::msg::PointStamped new_command;

    new_command.point.x = x_offset;
    new_command.point.y = y_offset;
    new_command.point.z = z_offset;

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
            new_command.point.x = new_command.point.x + (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0)* pow(-1, x_flip);
            break;
        case InputType::Trigger:
            new_command.point.x = new_command.point.x + (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0))* pow(-1, x_flip);
            break;
        case InputType::Button:
            new_command.point.x = new_command.point.x + (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index)* pow(-1, x_flip);
            break;
    }

    if (new_command.point.x >= curr_x_max)
    {
        new_command.point.x = curr_x_max;
    }
    if (new_command.point.x <= -curr_x_max)
    {
        new_command.point.x = -curr_x_max;
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
            new_command.point.x = new_command.point.x + (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0)* pow(-1, x_flip);
            break;
        case InputType::Trigger:
            new_command.point.x = new_command.point.x - (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0))* pow(-1, x_flip);
            break;
        case InputType::Button:
            new_command.point.x = new_command.point.x - (curr_x_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index)* pow(-1, x_flip);
            break;
    }

    if (new_command.point.x >= curr_x_max)
    {
        new_command.point.x = curr_x_max;
    }
    if (new_command.point.x <= -curr_x_max)
    {
        new_command.point.x = -curr_x_max;
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
            new_command.point.y = new_command.point.y - (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0) * pow(-1, y_flip);
            break;
        case InputType::Trigger:
            new_command.point.y = new_command.point.y - (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0)) * pow(-1, y_flip);
            break;
        case InputType::Button:
            new_command.point.y = new_command.point.y - (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index) * pow(-1, y_flip);
            break;
    }

    if (new_command.point.y >= curr_y_max)
    {
        new_command.point.y = curr_y_max;
    }
    if (new_command.point.y <= -curr_y_max)
    {
        new_command.point.y = -curr_y_max;
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
            new_command.point.y = new_command.point.y - (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) < 0 ? latest_joy_state.axes.at(input.index) : 0) * pow(-1, y_flip);
            break;
        case InputType::Trigger:
            new_command.point.y = new_command.point.y + (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0)) * pow(-1, y_flip);
            break;
        case InputType::Button:
            new_command.point.y = new_command.point.y + (curr_y_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index) * pow(-1, y_flip);
            break;
    }

    if (new_command.point.y >= curr_y_max)
    {
        new_command.point.y = curr_y_max;
    }
    if (new_command.point.y <= -curr_y_max)
    {
        new_command.point.y = -curr_y_max;
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
            new_command.point.z = new_command.point.z + (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0) * pow(-1, z_flip);
            break;
        case InputType::Trigger:
            new_command.point.z = new_command.point.z + (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0)) * pow(-1, z_flip);
            break;
        case InputType::Button:
            new_command.point.z = new_command.point.z + (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index) * pow(-1, z_flip);
            break;
    }

    if (new_command.point.z >= curr_z_max)
    {
        new_command.point.z = curr_z_max;
    }
    if (new_command.point.z <= -curr_z_max)
    {
        new_command.point.z = -curr_z_max;
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
            new_command.point.z = new_command.point.z + (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * (latest_joy_state.axes.at(input.index) > 0 ? latest_joy_state.axes.at(input.index) : 0) * pow(-1, z_flip);
            break;
        case InputType::Trigger:
            new_command.point.z = new_command.point.z - (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * (0.5 - (latest_joy_state.axes.at(input.index)/2.0)) * pow(-1, z_flip);
            break;
        case InputType::Button:
            new_command.point.z = new_command.point.z - (curr_z_max/rate_of_change_denom) * lin_rate_chg_fac * latest_joy_state.buttons.at(input.index) * pow(-1, z_flip);
            break;
    }

    if (new_command.point.z >= curr_z_max)
    {
        new_command.point.z = curr_z_max;
    }
    if (new_command.point.z <= -curr_z_max)
    {
        new_command.point.z = -curr_z_max;
    }


    return new_command;
}

static geometry_msgs::msg::PointStamped normalize_pntstmp(geometry_msgs::msg::PointStamped input_command, float new_mag)
{
    geometry_msgs::msg::PointStamped norm_command = input_command;
    float magnitude = sqrt(input_command.point.x * input_command.point.x + input_command.point.y * input_command.point.y + input_command.point.z * input_command.point.z);
    
    if (new_mag == 0)
    {
        new_mag = magnitude;
    }

    // If magnitude != 0 adjust pos accordingly, otherwise return original vector
    if (magnitude != 0)
    {
        norm_command.point.x = new_mag * norm_command.point.x / magnitude;
        norm_command.point.y = new_mag * norm_command.point.y / magnitude;
        norm_command.point.z = new_mag * norm_command.point.z / magnitude; 
    }

    return norm_command;
}

static geometry_msgs::msg::PointStamped add_pntstmp(geometry_msgs::msg::PointStamped add1, geometry_msgs::msg::PointStamped add2)
{
    geometry_msgs::msg::PointStamped new_command;
    new_command.point.x = add1.point.x + add2.point.x;
    new_command.point.y = add1.point.y + add2.point.y;
    new_command.point.z = add1.point.z + add2.point.z;
    return new_command;
}