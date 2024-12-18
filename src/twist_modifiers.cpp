#include "unified_teleop/twist_modifiers.hpp"

namespace rosnu
{
    geometry_msgs::msg::Vector3 set_vector3(double x, double y, double z)
    {
        geometry_msgs::msg::Vector3 new_vector3;
        
        new_vector3.x = x;
        new_vector3.y = y;
        new_vector3.z = z;

        return new_vector3;
    }

    geometry_msgs::msg::Vector3 invert_vector3(geometry_msgs::msg::Vector3 input_vector3, bool x_flip, bool y_flip, bool z_flip)
    {
        input_vector3.x *= pow(-1, x_flip);
        input_vector3.y *= pow(-1, y_flip);
        input_vector3.z *= pow(-1, z_flip);

        return input_vector3;
    }

    geometry_msgs::msg::Vector3 add_vector3(geometry_msgs::msg::Vector3 add1, geometry_msgs::msg::Vector3 add2)
    {
        geometry_msgs::msg::Vector3 new_vector3;
        new_vector3.x = add1.x + add2.x;
        new_vector3.y = add1.y + add2.y;
        new_vector3.z = add1.z + add2.z;
        return new_vector3;
    }

    geometry_msgs::msg::Vector3 subtract_vector3(geometry_msgs::msg::Vector3 subtracted, geometry_msgs::msg::Vector3 subtractor)
    {
        geometry_msgs::msg::Vector3 new_vector3;
        new_vector3.x = subtracted.x - subtractor.x;
        new_vector3.y = subtracted.y - subtractor.y;
        new_vector3.z = subtracted.z - subtractor.z;
        return new_vector3;
    }

    geometry_msgs::msg::Vector3 multiply_vector3(geometry_msgs::msg::Vector3 input_vector3, double x_mult, double y_mult, double z_mult)
    {
        input_vector3.x *= x_mult;
        input_vector3.y *= y_mult;
        input_vector3.z *= z_mult;

        return input_vector3;
    }

    geometry_msgs::msg::Vector3 normalize_vector3(geometry_msgs::msg::Vector3 input_vector3, double new_mag)
    {
        // Create a copy of the input Vector3 message to work with
        geometry_msgs::msg::Vector3 normalized_vector3 = input_vector3;
        
        // Calculate the current magnitude (length) of the vector represented by the Vector3 message
        // This is done using the Pythagorean theorem: sqrt(x^2 + y^2 + z^2)
        double magnitude = sqrt(input_vector3.x * input_vector3.x + 
                                input_vector3.y * input_vector3.y + 
                                input_vector3.z * input_vector3.z);
        
        // If the desired new magnitude (new_mag) is 0, then use the current magnitude
        // This check allows the function to simply normalize the vector without scaling it if new_mag is 0
        if (new_mag == 0)
        {
            new_mag = magnitude;
        }

        // Check to ensure the current magnitude is not 0 to avoid division by zero
        if (magnitude != 0)
        {
            // Adjust each component of the vector (x, y, z) to the new magnitude
            // The formula used is: (component / current magnitude) * new magnitude
            // This scales the vector to have a magnitude of new_mag while maintaining its direction
            normalized_vector3.x = new_mag * normalized_vector3.x / magnitude;
            normalized_vector3.y = new_mag * normalized_vector3.y / magnitude;
            normalized_vector3.z = new_mag * normalized_vector3.z / magnitude; 
        }

        // Return the normalized (and possibly scaled) Vector3 message
        return normalized_vector3;
    }

    geometry_msgs::msg::Vector3 round_vector3(geometry_msgs::msg::Vector3 input_command, int num_decimal_places)
    {
        geometry_msgs::msg::Vector3 new_command;
        new_command.x = round(input_command.x * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.y = round(input_command.y * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.z = round(input_command.z * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        return new_command;
    }

    geometry_msgs::msg::Vector3 adjust_mirror_joy(const std::optional<rosnu::MovementInput> input,
                                                        const geometry_msgs::msg::Vector3 orig_message,
                                                        const AxisType axis_type,
                                                        const bool is_increasing,
                                                        const sensor_msgs::msg::Joy joy_state)
    {
        // If the provided input is null (not initialized), return the original message without modifications
        if (!input)
        {
            return orig_message;
        }
        
        // Create a new Vector3 message starting as a copy of the original message
        geometry_msgs::msg::Vector3 new_message = orig_message;
        double reading = 0.0f; // Initialize a variable to store the controller input's value
        
        // Determine the type of the input (Axis, Trigger, Button) and read its value from the joy_state
        switch (input->type)
        {
            case rosnu::InputType::Axis:
                // For an axis input, directly take its reading
                reading = joy_state.axes.at(input->index);
                break;
            case rosnu::InputType::Trigger:
                // For a trigger, convert its value (normally -1.0 to 1.0) to a range of 0.0 to 1.0
                reading = 0.5 - (joy_state.axes.at(input->index)/2.0);
                break;
            case rosnu::InputType::Button:
                // For a button, take its binary state (pressed or not pressed)
                reading = joy_state.buttons.at(input->index);
                break;
            case rosnu::InputType::None:
                // Log an error and exit if the input type is uninitialized
                RCLCPP_ERROR(rclcpp::get_logger("pnt_stmp modifier"), "InputType is None");
                rclcpp::shutdown();
        }

        // Determine whether to adjust the value based on the reading and the intended direction (increase or decrease)
        if ((input->type == rosnu::InputType::Axis && ((is_increasing && reading > 0.0f) || (!is_increasing && reading < 0.0f))) ||
            (input->type != rosnu::InputType::Axis && reading != 0.0f))
        {
            double new_value = reading; // Store the modified value based on the reading
            // If decreasing the value for non-axis inputs, invert the sign of the new value
            if (input->type != rosnu::InputType::Axis && !is_increasing)
            {
                new_value = -new_value;
            }

            // Adjust the specified axis of the new Vector3 message with the new value
            switch (axis_type)
            {
                case AxisType::X_Axis:
                    new_message.x += new_value;
                    break;
                case AxisType::Y_Axis:
                    new_message.y += new_value;
                    break;
                case AxisType::Z_Axis:
                    new_message.z += new_value;
                    break;
            }
        }

        // Return the Vector3 message with the adjusted axis value
        return new_message;
    }
}