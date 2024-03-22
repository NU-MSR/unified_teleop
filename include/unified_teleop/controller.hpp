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
        Axis,
        Trigger,
        Button,
        None
    };

    /// @brief 
    constexpr InputType UNUSED_TYPE = InputType::None;

    /// @brief
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
            sensor_msgs::msg::Joy latest_joy_state;
            bool fresh_joy_state;
            std::map<std::string, int> button_map;
        
        public:
            //
            // CONSTRUCTORS
            //
            Controller();

            Controller(YAML::Node device_config_file_path);

            //
            // FUNCTIONS
            //

            /// @brief Updates the latest_joy_state with the provided message and sets fresh_joy_state to true.
            /// @param msg 
            /// @details This function updates the controller's input states and indicates that it is freshly (/newly) updated.
            void update_joy_state(const sensor_msgs::msg::Joy::SharedPtr &msg);

            /// @brief Using a provided directory path to an input device config file, this function reads the file and generates a map of input names to their respective index numbers.
            void update_button_map(YAML::Node device_config_file_path);

            /// @brief Using the input name and the map of input names to their respective index numbers provided by an input device config file, this function generates a MovementInput object.
            std::optional<MovementInput> generate_MovementInput(const std::string input_assignment);

            //
            // GETTERS
            //

            /// @brief By providing the name of an input, this function returns the type of input it is based on the first character of its name.
            /// @param input_name 
            InputType get_input_type(std::string input_name);

            /// @brief Return the latest state of the input with the provided index number.
            double get_input_state(int index);
    };
}

#endif
