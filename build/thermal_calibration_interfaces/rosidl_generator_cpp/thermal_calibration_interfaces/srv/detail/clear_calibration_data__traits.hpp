// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thermal_calibration_interfaces:srv/ClearCalibrationData.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__TRAITS_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thermal_calibration_interfaces/srv/detail/clear_calibration_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ClearCalibrationData_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: confirm
  {
    out << "confirm: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ClearCalibrationData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: confirm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirm: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ClearCalibrationData_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use thermal_calibration_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thermal_calibration_interfaces::srv::ClearCalibrationData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::ClearCalibrationData_Request & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>()
{
  return "thermal_calibration_interfaces::srv::ClearCalibrationData_Request";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>()
{
  return "thermal_calibration_interfaces/srv/ClearCalibrationData_Request";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ClearCalibrationData_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ClearCalibrationData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ClearCalibrationData_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use thermal_calibration_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const thermal_calibration_interfaces::srv::ClearCalibrationData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::ClearCalibrationData_Response & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>()
{
  return "thermal_calibration_interfaces::srv::ClearCalibrationData_Response";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>()
{
  return "thermal_calibration_interfaces/srv/ClearCalibrationData_Response";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::ClearCalibrationData>()
{
  return "thermal_calibration_interfaces::srv::ClearCalibrationData";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::ClearCalibrationData>()
{
  return "thermal_calibration_interfaces/srv/ClearCalibrationData";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::ClearCalibrationData>
  : std::integral_constant<
    bool,
    has_fixed_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>::value &&
    has_fixed_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>::value
  >
{
};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::ClearCalibrationData>
  : std::integral_constant<
    bool,
    has_bounded_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>::value &&
    has_bounded_size<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>::value
  >
{
};

template<>
struct is_service<thermal_calibration_interfaces::srv::ClearCalibrationData>
  : std::true_type
{
};

template<>
struct is_service_request<thermal_calibration_interfaces::srv::ClearCalibrationData_Request>
  : std::true_type
{
};

template<>
struct is_service_response<thermal_calibration_interfaces::srv::ClearCalibrationData_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__TRAITS_HPP_
