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
        previous_joy_state = current_joy_state;
        current_joy_state = *msg;
        fresh_joy_state = true;
    }

    void Controller::update_button_map(YAML::Node device_config_file_path)
    {
        // Indicate the name of the input device being used
        const auto device_name = device_config_file_path["name"].as<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("controller"), ("Currently using the " + device_name + " input device").c_str());
        // Fill in the button map with the button mapping from the device config file
        for (const auto& it : device_config_file_path["mapping"])
        {
            button_map[it.first.as<std::string>()] = it.second.as<int>();
        }
    }

    std::optional<MovementInput> Controller::generate_MovementInput(const std::string input_assignment)
    {
        if (input_assignment != "UNUSED")
        {
            MovementInput result_input;
            result_input.index = button_map.at(input_assignment);
            result_input.type = get_input_type(input_assignment);

            // Add the generated MovementInput object to the MovementInput_vec
            MovementInput_vec.push_back(result_input);

            return result_input;
        }
        else
        {
            return std::nullopt;
        }
    }

    double Controller::read_MovementInput(const std::optional<MovementInput> input, const sensor_msgs::msg::Joy joy_state)
    {
        if (input.has_value())
        {
            switch(input->type)
            {
                case InputType::Axis:
                    return joy_state.axes.at(input->index);
                case InputType::Trigger:
                    return joy_state.axes.at(input->index);
                case InputType::Button:
                    return joy_state.buttons.at(input->index);
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("controller"), "MovementInput not initialized");
                    return 0.0;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("controller"), "MovementInput not initialized");
            return 0.0;
        }
    }

    void Controller::clear_MovementInput_vec()
    {
        MovementInput_vec.clear();
    }

    bool Controller::is_joy_state_different()
    {
        bool result = false;
            for (int i = 0; i < static_cast<int>(MovementInput_vec.size()); i++)
            {
                std::optional<rosnu::MovementInput> input = MovementInput_vec[i];
                double old_val, new_val;

                // If the input is null, do not check it and continue to the next iteration
                if (!input)
                {
                    continue;
                }

                new_val = read_MovementInput(input, current_joy_state);
                old_val = read_MovementInput(input, previous_joy_state);

                // If any of the inputs are different, immediately break the loop
                if (old_val != new_val)
                {
                    result = true;
                    break;
                }
            }

            return result;
    }

    bool Controller::is_enabled(const std::optional<MovementInput> input)
    {
        if (always_enabled)
        {
            return true;
        }
        else if (input.has_value())
        {
            return std::abs(read_MovementInput(input, current_joy_state)) > 0.0;
        }
        else
        {
            return false;
        }
    }

    //
    // GETTERS
    //

    InputType Controller::get_input_type(const std::string input_name)
    {
        if (input_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("controller"), "Provided input is empty");
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
                RCLCPP_ERROR(rclcpp::get_logger("controller"), "Unable to determine input type");
                rclcpp::shutdown();
                throw std::runtime_error("Unable to determine input type");
        }
    }

    double Controller::get_input_state(int index)
    {
        return current_joy_state.axes.at(index);
    }

    sensor_msgs::msg::Joy Controller::get_current_joy_state()
    {
        return current_joy_state;
    }

    //
    // SETTERS
    //

    void Controller::set_always_enabled(bool enabled)
    {
        always_enabled = enabled;
    }
}