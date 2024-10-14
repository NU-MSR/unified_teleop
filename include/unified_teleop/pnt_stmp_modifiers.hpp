#ifndef PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP
#define PNT_STMP_MODIFIERS_INCLUDE_GUARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "unified_teleop/controller.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

namespace rosnu
{
    /// @brief Creates and returns a PointStamped message with specified x, y, z coordinates.
    /// @param x The x coordinate for the new PointStamped message.
    /// @param y The y coordinate for the new PointStamped message.
    /// @param z The z coordinate for the new PointStamped message.
    /// @return A PointStamped message with the given coordinates.
    geometry_msgs::msg::PointStamped set_pnt_stmp(double x, double y, double z);

    /// @brief Inverts the coordinates of a PointStamped message based on the specified flip flags for each axis.
    /// @param input_pnt_stmp The original PointStamped message to be inverted.
    /// @param x_flip A boolean flag indicating whether to invert the x axis.
    /// @param y_flip A boolean flag indicating whether to invert the y axis.
    /// @param z_flip A boolean flag indicating whether to invert the z axis.
    /// @return The inverted PointStamped message.
    geometry_msgs::msg::PointStamped invert_pnt_stmp(geometry_msgs::msg::PointStamped input_pnt_stmp, bool x_flip, bool y_flip, bool z_flip);

    /// @brief Calculates and returns the addition of two PointStamped messages.
    /// @param add1 The first PointStamped message to be added.
    /// @param add2 The second PointStamped message to be added.
    /// @return A PointStamped message representing the vector addition of add1 and add2.
    geometry_msgs::msg::PointStamped add_pntstmp(geometry_msgs::msg::PointStamped add1, geometry_msgs::msg::PointStamped add2);

    /// @brief Calculates and returns the subtraction of two PointStamped messages.
    /// @param subtracted The PointStamped message to be subtracted from.
    /// @param subtractor The PointStamped message to subtract.
    /// @return A PointStamped message representing the vector difference between subtracted and subtractor.
    geometry_msgs::msg::PointStamped subtract_pntstmp(geometry_msgs::msg::PointStamped subtracted, geometry_msgs::msg::PointStamped subtractor);

    /// @brief Multiplies the coordinates of a PointStamped message by specified multipliers.
    /// @param input_pnt_stmp The PointStamped message to be multiplied.
    /// @param x_mult The multiplier for the x axis.
    /// @param y_mult The multiplier for the y axis.
    /// @param z_mult The multiplier for the z axis.
    /// @return The PointStamped message after applying the multipliers to its coordinates.
    geometry_msgs::msg::PointStamped multiply_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double x_mult, double y_mult, double z_mult);

    /// @brief Normalizes the coordinates of a PointStamped message to a specified magnitude.
    /// @param input_pnt_stmp The PointStamped message to be normalized.
    /// @param new_mag The desired magnitude for the normalized PointStamped message.
    /// @return The normalized PointStamped message with the specified magnitude.
    geometry_msgs::msg::PointStamped normalize_pntstmp(geometry_msgs::msg::PointStamped input_pnt_stmp, double new_mag);

    /// @brief Rounds the coordinates of a PointStamped message to a specified number of decimal places.
    /// @param input_command The PointStamped message whose coordinates will be rounded.
    /// @param num_decimal_places The number of decimal places to round to.
    /// @return The PointStamped message with rounded coordinates.
    geometry_msgs::msg::PointStamped round_pntstmp(geometry_msgs::msg::PointStamped input_command, int num_decimal_places);

    /// @brief The specific axis in a PointStamped message that will be modified
    enum class AxisType
    {
        X_Axis,
        Y_Axis,
        Z_Axis
    };

    /// @brief Adjusts the value of a specific axis of a PointStamped message based on controller input, increasing or decreasing the value as specified.
    /// @param input The controller input that determines how the original message will be modified.
    /// @param orig_message The original PointStamped message to be modified.
    /// @param axis_type The axis (X, Y, Z) of the PointStamped message to be adjusted.
    /// @param is_increasing Boolean indicating whether to increase (true) or decrease (false) the axis value.
    /// @param joy_state The current state of the controller inputs.
    /// @return A modified PointStamped message with adjusted values for the specified axis.
    geometry_msgs::msg::PointStamped adjust_mirror_joy(const std::optional<rosnu::MovementInput> input,
                                                        const geometry_msgs::msg::PointStamped orig_message,
                                                        const AxisType axis_type,
                                                        const bool is_increasing,
                                                        const sensor_msgs::msg::Joy joy_state);
}

#endif
