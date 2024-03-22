#include "unified_teleop/controller.hpp"

namespace rosnu
{
    //
    // CONSTRUCTORS
    //

    Controller::Controller()
    {
        fresh_joy_state = false;
    }

    Controller::Controller(YAML::Node device_config_file_path)
    {
        fresh_joy_state = false;
        update_button_map(device_config_file_path);
    }

    //
    // FUNCTIONS
    //

    void Controller::update_joy_state(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        latest_joy_state = *msg;
        fresh_joy_state = true;
    }

    void Controller::update_button_map(YAML::Node device_config_file_path)
    {
        // Indicate the name of the input device being used
        const auto device_name = device_config_file_path["name"].as<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("point_stamped_mirror"), ("Currently using the " + device_name + " input device").c_str());
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

            return result_input;
        }
        else
        {
            return std::nullopt;
        }
    }

    //
    // GETTERS
    //

    InputType Controller::get_input_type(const std::string input_name)
    {
        if (input_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("point_stamped_mirror"), "Provided input is empty");
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
                RCLCPP_ERROR(rclcpp::get_logger("point_stamped_mirror"), "Unable to determine input type");
                rclcpp::shutdown();
                throw std::runtime_error("Unable to determine input type");
        }
    }

    double Controller::get_input_state(int index)
    {
        return latest_joy_state.axes.at(index);
    }
}