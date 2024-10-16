#ifndef PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP
#define PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "unified_teleop/controller.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

namespace rosnu
{
    // NOTE the Twist message comprises of two Vector3 messages, linear and angular, hence these modifiers will be for Vector3
    //      and can be used interchangably for the linear and angular components of the Twist message

    /// @brief Creates and returns a Vector3 message with specified x, y, z coordinates.
    /// @param x The x coordinate for the new Vector3 message.
    /// @param y The y coordinate for the new Vector3 message.
    /// @param z The z coordinate for the new Vector3 message.
    /// @return A Vector3 message with the given coordinates.
    geometry_msgs::msg::Vector3 set_vector3(double x, double y, double z);

    /// @brief Inverts the coordinates of a Vector3 message based on the specified flip flags for each axis.
    /// @param input_vector3 The original Vector3 message to be inverted.
    /// @param x_flip A boolean flag indicating whether to invert the x axis.
    /// @param y_flip A boolean flag indicating whether to invert the y axis.
    /// @param z_flip A boolean flag indicating whether to invert the z axis.
    /// @return The inverted Vector3 message.
    geometry_msgs::msg::Vector3 invert_vector3(geometry_msgs::msg::Vector3 input_vector3, bool x_flip, bool y_flip, bool z_flip);

    /// @brief Calculates and returns the addition of two Vector3 messages.
    /// @param add1 The first Vector3 message to be added.
    /// @param add2 The second Vector3 message to be added.
    /// @return A Vector3 message representing the vector addition of add1 and add2.
    geometry_msgs::msg::Vector3 add_vector3(geometry_msgs::msg::Vector3 add1, geometry_msgs::msg::Vector3 add2);

    /// @brief Calculates and returns the subtraction of two Vector3 messages.
    /// @param subtracted The Vector3 message to be subtracted from.
    /// @param subtractor The Vector3 message to subtract.
    /// @return A Vector3 message representing the vector difference between subtracted and subtractor.
    geometry_msgs::msg::Vector3 subtract_vector3(geometry_msgs::msg::Vector3 subtracted, geometry_msgs::msg::Vector3 subtractor);

    /// @brief Multiplies the coordinates of a Vector3 message by specified multipliers.
    /// @param input_vector3 The Vector3 message to be multiplied.
    /// @param x_mult The multiplier for the x axis.
    /// @param y_mult The multiplier for the y axis.
    /// @param z_mult The multiplier for the z axis.
    /// @return The Vector3 message after applying the multipliers to its coordinates.
    geometry_msgs::msg::Vector3 multiply_vector3(geometry_msgs::msg::Vector3 input_vector3, double x_mult, double y_mult, double z_mult);

    /// @brief Normalizes the coordinates of a Vector3 message to a specified magnitude.
    /// @param input_vector3 The Vector3 message to be normalized.
    /// @param new_mag The desired magnitude for the normalized Vector3 message.
    /// @return The normalized Vector3 message with the specified magnitude.
    geometry_msgs::msg::Vector3 normalize_vector3(geometry_msgs::msg::Vector3 input_vector3, double new_mag);

    /// @brief Rounds the coordinates of a Vector3 message to a specified number of decimal places.
    /// @param input_command The Vector3 message whose coordinates will be rounded.
    /// @param num_decimal_places The number of decimal places to round to.
    /// @return The Vector3 message with rounded coordinates.
    geometry_msgs::msg::Vector3 round_vector3(geometry_msgs::msg::Vector3 input_command, int num_decimal_places);

    /// @brief The specific axis in a Vector3 message that will be modified
    enum class AxisType
    {
        X_Axis,
        Y_Axis,
        Z_Axis
    };

    /// @brief Adjusts the value of a specific axis of a Vector3 message based on controller input, increasing or decreasing the value as specified.
    /// @param input The controller input that determines how the original message will be modified.
    /// @param orig_message The original Vector3 message to be modified.
    /// @param axis_type The axis (X, Y, Z) of the Vector3 message to be adjusted.
    /// @param is_increasing Boolean indicating whether to increase (true) or decrease (false) the axis value.
    /// @param joy_state The current state of the controller inputs.
    /// @return A modified Vector3 message with adjusted values for the specified axis.
    geometry_msgs::msg::Vector3 adjust_mirror_joy(const std::optional<rosnu::MovementInput> input,
                                                        const geometry_msgs::msg::Vector3 orig_message,
                                                        const AxisType axis_type,
                                                        const bool is_increasing,
                                                        const sensor_msgs::msg::Joy joy_state);
}

#endif
