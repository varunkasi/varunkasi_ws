// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__TRAITS_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thermal_calibration_interfaces/msg/detail/calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace thermal_calibration_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CalibrationModel & msg,
  std::ostream & out)
{
  out << "{";
  // member: model_type
  {
    out << "model_type: ";
    rosidl_generator_traits::value_to_yaml(msg.model_type, out);
    out << ", ";
  }

  // member: degree
  {
    out << "degree: ";
    rosidl_generator_traits::value_to_yaml(msg.degree, out);
    out << ", ";
  }

  // member: parameters
  {
    if (msg.parameters.size() == 0) {
      out << "parameters: []";
    } else {
      out << "parameters: [";
      size_t pending_items = msg.parameters.size();
      for (auto item : msg.parameters) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: r_squared
  {
    out << "r_squared: ";
    rosidl_generator_traits::value_to_yaml(msg.r_squared, out);
    out << ", ";
  }

  // member: rmse
  {
    out << "rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.rmse, out);
    out << ", ";
  }

  // member: points_count
  {
    out << "points_count: ";
    rosidl_generator_traits::value_to_yaml(msg.points_count, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: raw_value_range
  {
    if (msg.raw_value_range.size() == 0) {
      out << "raw_value_range: []";
    } else {
      out << "raw_value_range: [";
      size_t pending_items = msg.raw_value_range.size();
      for (auto item : msg.raw_value_range) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrationModel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: model_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "model_type: ";
    rosidl_generator_traits::value_to_yaml(msg.model_type, out);
    out << "\n";
  }

  // member: degree
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "degree: ";
    rosidl_generator_traits::value_to_yaml(msg.degree, out);
    out << "\n";
  }

  // member: parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.parameters.size() == 0) {
      out << "parameters: []\n";
    } else {
      out << "parameters:\n";
      for (auto item : msg.parameters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r_squared
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_squared: ";
    rosidl_generator_traits::value_to_yaml(msg.r_squared, out);
    out << "\n";
  }

  // member: rmse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.rmse, out);
    out << "\n";
  }

  // member: points_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "points_count: ";
    rosidl_generator_traits::value_to_yaml(msg.points_count, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: raw_value_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.raw_value_range.size() == 0) {
      out << "raw_value_range: []\n";
    } else {
      out << "raw_value_range:\n";
      for (auto item : msg.raw_value_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrationModel & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace thermal_calibration_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use thermal_calibration_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thermal_calibration_interfaces::msg::CalibrationModel & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::msg::CalibrationModel & msg)
{
  return thermal_calibration_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::msg::CalibrationModel>()
{
  return "thermal_calibration_interfaces::msg::CalibrationModel";
}

template<>
inline const char * name<thermal_calibration_interfaces::msg::CalibrationModel>()
{
  return "thermal_calibration_interfaces/msg/CalibrationModel";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::msg::CalibrationModel>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::msg::CalibrationModel>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::msg::CalibrationModel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__TRAITS_HPP_
