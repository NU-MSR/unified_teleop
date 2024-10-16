#include "unified_teleop/controller.hpp"

namespace rosnu
{
    //
    // CONSTRUCTORS
    //

    Controller::Controller() {};

    Controller::Controller(YAML::Node device_config_file_path)
    {
        update_button_map(device_config_file_path);
    }

    //
    // FUNCTIONS
    //

    void Controller::update_joy_state(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        previous_joy_state = current_joy_state; // Store the current state as the previous state before updating.
        current_joy_state = *msg; // Update the current state with the new message.

        // If it's the first joy state, set the previous state to the updated current state to prevent error.
        if (first_joy_state)
        {
            previous_joy_state = current_joy_state;
            first_joy_state = false;
        }
    }

    void Controller::update_button_map(YAML::Node device_config_file_path)
    {
        // Log the input device being used for clarity.
        const auto device_name = device_config_file_path["name"].as<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("controller"), ("Currently using the " + device_name + " input device").c_str());
        
        // Populate the button map with mappings from the device configuration.
        for (const auto& it : device_config_file_path["mapping"])
        {
            button_map[it.first.as<std::string>()] = it.second.as<int>();
        }
    }

    std::optional<MovementInput> Controller::generate_MovementInput(const std::string input_assignment)
    {
        if (input_assignment != "UNUSED")
        {
            MovementInput result_input(button_map.at(input_assignment), get_input_type(input_assignment));  // Set index based on the button map
                                                                                                            // and determine the type of input.
                                                                                                            
            // Add the newly generated MovementInput object to the MovementInput_vec for tracking.
            MovementInput_vec.push_back(result_input);

            return result_input; // Return the MovementInput object wrapped in an optional.
        }
        else
        {
            return std::nullopt; // Return nullopt if the input is unused.
        }
    }

    double Controller::read_MovementInput(const std::optional<MovementInput> input, const sensor_msgs::msg::Joy joy_state) const
    {
        if (input.has_value())
        {
            // Return the state of the input based on its type (Axis, Trigger, or Button).
            switch(input->type)
            {
                case InputType::Axis:
                case InputType::Trigger: // Treat Trigger inputs as Axis for simplicity.
                    return joy_state.axes.at(input->index); // Return the axis value.
                case InputType::Button:
                    return joy_state.buttons.at(input->index); // Return the button state (pressed or not).
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("controller"), "MovementInput type is not recognized");
                    return 0.0; // Return default value if the input type is unrecognized.
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("controller"), "MovementInput object is not initialized");
            return 0.0; // Return default value if the MovementInput object is not initialized.
        }
    }

    void Controller::clear_MovementInput_vec()
    {
        MovementInput_vec.clear(); // Empty the vector of MovementInput objects.
    }

    bool Controller::is_joy_state_different() const
    {
        bool result = false;
        for (int i = 0; i < static_cast<int>(MovementInput_vec.size()); i++)
        {
            std::optional<rosnu::MovementInput> input = MovementInput_vec[i];
            double old_val, new_val;

            // Skip comparison if the input is null.
            if (!input)
            {
                continue;
            }

            // Read the old and new values for the given input.
            new_val = read_MovementInput(input, current_joy_state);
            old_val = read_MovementInput(input, previous_joy_state);

            // Check for any difference in input states.
            if (old_val != new_val)
            {
                result = true; // Set result to true if any input has changed.
                break; // Exit the loop as we have found a difference.
            }
        }

        return result; // Return whether any inputs have changed.
    }

    bool Controller::is_enabled(const std::optional<MovementInput> input) const
    {
        // Check the always_enabled flag first.
        if (always_enabled)
        {
            return true; // Return true if always_enabled is set.
        }
        else if (input.has_value())
        {
            // Check if the input's state is non-zero, indicating activity.
            return std::abs(read_MovementInput(input, current_joy_state)) > 0.0;
        }
        else
        {
            return false; // Return false if the input is not initialized or its state is zero.
        }
    }

    InputType Controller::get_input_type(const std::string input_name) const
    {
        // Handle empty input names.
        if (input_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("controller"), "Provided input name is empty");
            throw std::runtime_error("Provided input name is empty");
        }

        // Determine the input type based on the first character of the input name.
        char first_character = input_name.at(0);
        
        switch (first_character)
        {
            case 'a':
                return InputType::Axis; // Return Axis if the name starts with 'a'.
            case 't':
                return InputType::Trigger; // Return Trigger if the name starts with 't'.
            case 'b':
                return InputType::Button; // Return Button if the name starts with 'b'.
            default:
                RCLCPP_ERROR(rclcpp::get_logger("controller"), "Unable to determine the input type from the name");
                throw std::runtime_error("Unable to determine the input type from the name");
        }
    }

    sensor_msgs::msg::Joy Controller::get_current_joy_state() const
    {
        return current_joy_state; // Return the stored current joy state.
    }

    //
    // SETTERS
    //

    void Controller::set_always_enabled(bool enabled)
    {
        always_enabled = enabled; // Update the always_enabled flag with the given value.
    }
}