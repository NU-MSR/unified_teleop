#ifndef PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP
#define PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "unified_teleop/controller.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

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

    /// @brief Returns a PointStamped message that reflects the sum between two given PointStamped messages
    /// @param subtracted - First part of PointStamped addition
    /// @param subtractor - Second part of the PointStamped addition
    geometry_msgs::msg::PointStamped add_pntstmp(geometry_msgs::msg::PointStamped add1, geometry_msgs::msg::PointStamped add2)
    {
        geometry_msgs::msg::PointStamped new_pnt_stmp;
        new_pnt_stmp.point.x = add1.point.x + add2.point.x;
        new_pnt_stmp.point.y = add1.point.y + add2.point.y;
        new_pnt_stmp.point.z = add1.point.z + add2.point.z;
        return new_pnt_stmp;
    }

    /// @brief Returns a PointStamped message that reflects the difference between two given PointStamped messages
    /// @param subtracted - The PointStamped message that will be subtracted
    /// @param subtractor - The PointStamped message that will be subtracting from the subtracted
    geometry_msgs::msg::PointStamped subtract_pntstmp(geometry_msgs::msg::PointStamped subtracted, geometry_msgs::msg::PointStamped subtractor)
    {
        geometry_msgs::msg::PointStamped new_pnt_stmp;
        new_pnt_stmp.point.x = subtracted.point.x - subtractor.point.x;
        new_pnt_stmp.point.y = subtracted.point.y - subtractor.point.y;
        new_pnt_stmp.point.z = subtracted.point.z - subtractor.point.z;
        return new_pnt_stmp;
    }

    /// @brief Returns a PointStamped message that reflects the difference between two given PointStamped messages
    /// @param subtracted - The PointStamped message that will be subtracted
    /// @param subtractor - The PointStamped message that will be subtracting from the subtracted
    geometry_msgs::msg::PointStamped multiply_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double x_mult, double y_mult, double z_mult)
    {
        input_pnt_stmp.point.x *= x_mult;
        input_pnt_stmp.point.y *= y_mult;
        input_pnt_stmp.point.z *= z_mult;

        return input_pnt_stmp;
    }

    /// @brief Returns a PointStamped message that normalizes the values to a given magnitude
    /// @param input_pnt_stmp - The PointStamped message that will be normalized
    /// @param new_mag - The desired magnitude for the resulting PointStamped message
    geometry_msgs::msg::PointStamped normalize_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double new_mag)
    {
        geometry_msgs::msg::PointStamped normalized_pnt_stmp = input_pnt_stmp;
        double magnitude = sqrt(input_pnt_stmp.point.x * input_pnt_stmp.point.x + input_pnt_stmp.point.y * input_pnt_stmp.point.y + input_pnt_stmp.point.z * input_pnt_stmp.point.z);
        
        if (new_mag == 0)
        {
            new_mag = magnitude;
        }

        // If magnitude != 0 adjust pos accordingly, otherwise return original vector
        if (magnitude != 0)
        {
            normalized_pnt_stmp.point.x = new_mag * normalized_pnt_stmp.point.x / magnitude;
            normalized_pnt_stmp.point.y = new_mag * normalized_pnt_stmp.point.y / magnitude;
            normalized_pnt_stmp.point.z = new_mag * normalized_pnt_stmp.point.z / magnitude; 
        }

        return normalized_pnt_stmp;
    }

    /// @brief Returns a PointStamped message that with rounded values to a certain decimal point
    /// @param input_command - The PointStamped message that will be rounded
    geometry_msgs::msg::PointStamped round_pntstmp(geometry_msgs::msg::PointStamped input_command, int num_decimal_places)
    {
        geometry_msgs::msg::PointStamped new_command;
        new_command.point.x = round(input_command.point.x * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.point.y = round(input_command.point.y * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        new_command.point.z = round(input_command.point.z * std::pow(10, num_decimal_places))/std::pow(10, num_decimal_places);
        return new_command;
    }

    /// @brief The specific axis in a PointStamped message that will be modified
    enum class AxisType
    {
        X_Axis,
        Y_Axis,
        Z_Axis
    };

    /// @brief Returns a modified message with a increased or decreased value for a specific axis depending
    /// on what the user defines in both calling this function and parameters assigned upon startup
    /// @param input - The controller input that decides how the original message will be modified
    /// @param orig_message - The message that will be modified with new values
    /// @param axis_type - The directional axis of the message that will be modified
    /// @param is_increasing - Whethe the modification will involve increasing or decreasing the value
    /// @param joy_state - The current state of the controller inputs
    geometry_msgs::msg::PointStamped adjust_mirror_joy(const std::optional<rosnu::MovementInput> input,
                                                        const geometry_msgs::msg::PointStamped orig_message,
                                                        const AxisType axis_type,
                                                        const bool is_increasing,
                                                        const sensor_msgs::msg::Joy joy_state)
    {
        // If the input is null, then return the original message
        if (!input)
        {
            return orig_message;
        }
        
        // Initialize variables
        geometry_msgs::msg::PointStamped new_message = orig_message;
        double reading = 0.0f;
        
        // Based on the input type, take in the appropriate readings from the joy_state
        switch (input->type)
        {
            case rosnu::InputType::Axis:
                reading = joy_state.axes.at(input->index);
                break;
            case rosnu::InputType::Trigger:
                reading = 0.5 - (joy_state.axes.at(input->index)/2.0);
                break;
            case rosnu::InputType::Button:
                reading = joy_state.buttons.at(input->index);
                break;
            case rosnu::InputType::None:
                RCLCPP_ERROR(rclcpp::get_logger("pnt_stmp modifier"), "InputType is None");
                rclcpp::shutdown();
        }

        // If the input type is Axis, check reading based on whether it's above or below 0.0 and is_increasing
        // OR if the input type is not Axis, see if the reading is not 0.0
        if ((input->type == rosnu::InputType::Axis && ((is_increasing && reading > 0.0f) || (!is_increasing && reading < 0.0f))) ||
            (input->type != rosnu::InputType::Axis && reading != 0.0f))
        {
            // If the condition is satisfied, calculate the new_value for new_message
            double new_value = reading;
            // If the input type is not Axis AND the value is to be decreased (not increased),
            // then flip the sign of the new_value
            if (input->type != rosnu::InputType::Axis && !is_increasing)
            {
                new_value = -new_value;
            }

            // Based on axis specified to be modified, modify that aspect of the new PointStamped message
            switch (axis_type)
            {
                case AxisType::X_Axis:
                    new_message.point.x = new_value;
                    break;
                case AxisType::Y_Axis:
                    new_message.point.y = new_value;
                    break;
                case AxisType::Z_Axis:
                    new_message.point.z = new_value;
                    break;
            }
        }

        return new_message;
    }
}

#endif
