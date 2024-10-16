#include "unified_teleop/pnt_stmp_modifiers.hpp"

namespace rosnu
{
    geometry_msgs::msg::PointStamped set_pnt_stmp(double x, double y, double z)
    {
        geometry_msgs::msg::PointStamped new_pnt_stmp;
        
        new_pnt_stmp.point.x = x;
        new_pnt_stmp.point.y = y;
        new_pnt_stmp.point.z = z;

        return new_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped invert_pnt_stmp(geometry_msgs::msg::PointStamped input_pnt_stmp, bool x_flip, bool y_flip, bool z_flip)
    {
        input_pnt_stmp.point.x *= pow(-1, x_flip);
        input_pnt_stmp.point.y *= pow(-1, y_flip);
        input_pnt_stmp.point.z *= pow(-1, z_flip);

        return input_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped add_pntstmp(geometry_msgs::msg::PointStamped add1, geometry_msgs::msg::PointStamped add2)
    {
        geometry_msgs::msg::PointStamped new_pnt_stmp;
        new_pnt_stmp.point.x = add1.point.x + add2.point.x;
        new_pnt_stmp.point.y = add1.point.y + add2.point.y;
        new_pnt_stmp.point.z = add1.point.z + add2.point.z;
        return new_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped subtract_pntstmp(geometry_msgs::msg::PointStamped subtracted, geometry_msgs::msg::PointStamped subtractor)
    {
        geometry_msgs::msg::PointStamped new_pnt_stmp;
        new_pnt_stmp.point.x = subtracted.point.x - subtractor.point.x;
        new_pnt_stmp.point.y = subtracted.point.y - subtractor.point.y;
        new_pnt_stmp.point.z = subtracted.point.z - subtractor.point.z;
        return new_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped multiply_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double x_mult, double y_mult, double z_mult)
    {
        input_pnt_stmp.point.x *= x_mult;
        input_pnt_stmp.point.y *= y_mult;
        input_pnt_stmp.point.z *= z_mult;

        return input_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped normalize_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double new_mag)
    {
        // Create a copy of the input PointStamped message to work with
        geometry_msgs::msg::PointStamped normalized_pnt_stmp = input_pnt_stmp;
        
        // Calculate the current magnitude (length) of the vector represented by the PointStamped message
        // This is done using the Pythagorean theorem: sqrt(x^2 + y^2 + z^2)
        double magnitude = sqrt(input_pnt_stmp.point.x * input_pnt_stmp.point.x + 
                                input_pnt_stmp.point.y * input_pnt_stmp.point.y + 
                                input_pnt_stmp.point.z * input_pnt_stmp.point.z);
        
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
            normalized_pnt_stmp.point.x = new_mag * normalized_pnt_stmp.point.x / magnitude;
            normalized_pnt_stmp.point.y = new_mag * normalized_pnt_stmp.point.y / magnitude;
            normalized_pnt_stmp.point.z = new_mag * normalized_pnt_stmp.point.z / magnitude; 
        }

        // Return the normalized (and possibly scaled) PointStamped message
        return normalized_pnt_stmp;
    }

    geometry_msgs::msg::PointStamped round_pntstmp(geometry_msgs::msg::PointStamped input_command, int num_decimal_places)
    {
        geometry_msgs::msg::PointStamped new_command;
        new_command.point.x = round(input_command.point.x * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.point.y = round(input_command.point.y * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.point.z = round(input_command.point.z * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        return new_command;
    }

    geometry_msgs::msg::PointStamped adjust_mirror_joy(const std::optional<rosnu::MovementInput> input,
                                                        const geometry_msgs::msg::PointStamped orig_message,
                                                        const AxisType axis_type,
                                                        const bool is_increasing,
                                                        const sensor_msgs::msg::Joy joy_state)
    {
        // If the provided input is null (not initialized), return the original message without modifications
        if (!input)
        {
            return orig_message;
        }
        
        // Create a new PointStamped message starting as a copy of the original message
        geometry_msgs::msg::PointStamped new_message = orig_message;
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
                throw std::runtime_error("InputType is None");
            default:
                // Log an error and exit if the input type is unrecognized
                throw std::runtime_error("InputType is not recognized");
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

            // Adjust the specified axis of the new PointStamped message with the new value
            switch (axis_type)
            {
                case AxisType::X_Axis:
                    new_message.point.x += new_value;
                    break;
                case AxisType::Y_Axis:
                    new_message.point.y += new_value;
                    break;
                case AxisType::Z_Axis:
                    new_message.point.z += new_value;
                    break;
                default:
                    // Log an error and exit if the axis type is unrecognized
                    throw std::runtime_error("AxisType is not recognized");
            }
        }

        // Return the PointStamped message with the adjusted axis value
        return new_message;
    }
}