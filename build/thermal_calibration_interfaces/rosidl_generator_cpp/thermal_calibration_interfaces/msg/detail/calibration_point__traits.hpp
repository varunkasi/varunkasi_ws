// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__TRAITS_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thermal_calibration_interfaces/msg/detail/calibration_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace thermal_calibration_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CalibrationPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: raw_value
  {
    out << "raw_value: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_value, out);
    out << ", ";
  }

  // member: reference_temp
  {
    out << "reference_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_temp, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrationPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: raw_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_value: ";
    rosidl_generator_traits::value_to_yaml(msg.raw_value, out);
    out << "\n";
  }

  // member: reference_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_temp, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrationPoint & msg, bool use_flow_style = false)
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
  const thermal_calibration_interfaces::msg::CalibrationPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::msg::CalibrationPoint & msg)
{
  return thermal_calibration_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::msg::CalibrationPoint>()
{
  return "thermal_calibration_interfaces::msg::CalibrationPoint";
}

template<>
inline const char * name<thermal_calibration_interfaces::msg::CalibrationPoint>()
{
  return "thermal_calibration_interfaces/msg/CalibrationPoint";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::msg::CalibrationPoint>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::msg::CalibrationPoint>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::msg::CalibrationPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__TRAITS_HPP_
