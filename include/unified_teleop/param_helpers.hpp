// param_helpers.hpp
#ifndef ROSNU_HELPERS_HPP
#define ROSNU_HELPERS_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

/// Additional parameter helper functions developed my NU MSR
namespace rosnu
{
    /// @brief declare a parameter without a default value. If the value is not set externally,
    /// an exception will be thrown when trying to get_param for this parameter.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param node - node for which the parameter is declared
    /// @param desc - (optional) the parameter description
    /// @throwD
    ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
    ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
    template <class T>
    void declare_param(const std::string &name, rclcpp::Node &node, const std::string &desc = "")
    {
        // init descriptor object and fill in description
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = desc;

        // declare parameter without a default value
        node.declare_parameter<T>(name, descriptor);
    }

    /// @brief declare a parameter with a default value.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param def - the default parameter value
    /// @param node - node for which the parameter is declared
    /// @param desc - (optional) the parameter description
    /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
    template <class T>
    void declare_param(const std::string &name, const T &def, rclcpp::Node &node, const std::string &desc = "")
    {
        // init descriptor object and fill in description
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = desc;

        // declare node with default value
        node.declare_parameter<T>(name, def, descriptor);
    }

    /// @brief get the value of a parameter.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param node - node for which the parameter was declared
    /// @return value of the parameter
    /// @throw rclcpp::exceptions::ParameterNotDeclaredException if the parameter has not been declared
    template <class T>
    T get_param(const std::string &name, rclcpp::Node &node)
    {
        return node.get_parameter(name).get_parameter_value().get<T>();
    }

    /// @brief declare a parameter without a default value and return the parameter value.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param node - node for which the parameter is declared
    /// @param desc - (optional) the parameter description
    /// @return value of the parameter
    /// @throw
    ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
    ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
    template <class T>
    T declare_and_get_param(const std::string &name, rclcpp::Node &node, const std::string &desc = "")
    {
        declare_param<T>(name, node, desc);
        return get_param<T>(name, node);
    }

    /// @brief declare a parameter with a default value and return the parameter value.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param def - the default parameter value
    /// @param node - node for which the parameter is declared
    /// @param desc - (optional) the parameter description
    /// @return value of the parameter
    /// @throw rclcpp::exceptions::ParameterAlreadyDeclaredException if the parameter has already been declared
    template <class T>
    T declare_and_get_param(const std::string &name, const T &def, rclcpp::Node &node, const std::string &desc = "")
    {
        declare_param<T>(name, def, node, desc);
        return get_param<T>(name, node);
    }
}

#endif // PARAM_HELPERS_HPP