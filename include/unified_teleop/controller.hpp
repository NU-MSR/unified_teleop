#ifndef TELEOP_CONTROLLER_INCLUDE_GUARD_HPP
#define TELEOP_CONTROLLER_INCLUDE_GUARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <optional>
#include <map>
#include <string>
#include <cmath>
#include "yaml-cpp/yaml.h"

namespace rosnu
{
    enum class InputType
    {
        /// @brief  Represents an axis input, such as a joystick; the values usually start from 0.0 and range from -1.0 to 1.0.
        Axis,
        /// @brief Represents a trigger input, such as a trigger button; the values usually start from 1.0 and range from -1.0 to 1.0.
        Trigger,
        /// @brief Represents a button input, such as a regular button; the values are usually either 0 or 1.
        Button,
        None
    };

    /// @brief Represents an unused input type.
    constexpr InputType UNUSED_TYPE = InputType::None;

    /// @brief Represents an unused index value.
    constexpr int UNUSED_INDEX = -1;

    struct MovementInput
    {
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

    class Controller
    {
        private:
            sensor_msgs::msg::Joy current_joy_state, previous_joy_state;
            bool always_enabled = false;
            bool first_joy_state = true;
            std::map<std::string, int> button_map;
            std::vector<std::optional<rosnu::MovementInput>> MovementInput_vec;
        
        public:
            //
            // CONSTRUCTORS
            //
            /// @brief Default constructor for Controller.
            Controller();

            /// @brief Constructor that initializes the Controller with device configuration.
            /// @param device_config_file_path YAML Node containing the device configuration file path.
            Controller(YAML::Node device_config_file_path);

            //
            // FUNCTIONS
            //

            /// @brief Reads and updates the current joystick state.
            /// @param msg The latest joystick message received.
            void update_joy_state(const sensor_msgs::msg::Joy::SharedPtr &msg);

            /// @brief Reads the device configuration file and updates the button map accordingly.
            /// @param device_config_file_path YAML Node containing the device configuration file path.
            void update_button_map(YAML::Node device_config_file_path);

            /// @brief Generates a MovementInput object based on the input name and current button map.
            /// @param input_assignment The name of the input to generate a MovementInput for.
            /// @return An optional MovementInput object.
            std::optional<MovementInput> generate_MovementInput(const std::string input_assignment);

            /// @brief Reads the state of a given input from a given joy state.
            /// @param input The MovementInput object to read the state for.
            /// @param joy_state The provided joy state.
            /// @return The state of the input as a double.
            double read_MovementInput(const std::optional<MovementInput> input, const sensor_msgs::msg::Joy joy_state);

            /// @brief Clears the vector of MovementInput objects.
            void clear_MovementInput_vec();

            /// @brief Checks if the current joy state is different from the previous one.
            /// @return True if different, otherwise false.
            bool is_joy_state_different();

            /// @brief Checks if a given input is enabled.
            /// @param input The MovementInput object to check.
            /// @return True if the input is enabled or if the variable always_enabled is true, otherwise false.
            bool is_enabled(const std::optional<rosnu::MovementInput> input);

            //
            // GETTERS
            //

            /// @brief Gets the input type based on the input name.
            /// @param input_name The name of the input.
            /// @return The InputType of the given input name.
            InputType get_input_type(std::string input_name);

            /// @brief Gets the current joystick state.
            /// @return The current joy state.
            sensor_msgs::msg::Joy get_current_joy_state();

            //
            // SETTERS
            //

            /// @brief Sets the always_enabled flag.
            /// @param enabled The new value for the always_enabled flag.
            void set_always_enabled(bool enabled);
    };
}

#endif
