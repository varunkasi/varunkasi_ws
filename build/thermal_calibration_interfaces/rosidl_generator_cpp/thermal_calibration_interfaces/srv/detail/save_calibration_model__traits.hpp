// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thermal_calibration_interfaces:srv/SaveCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__TRAITS_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thermal_calibration_interfaces/srv/detail/save_calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveCalibrationModel_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveCalibrationModel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveCalibrationModel_Request & msg, bool use_flow_style = false)
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
  const thermal_calibration_interfaces::srv::SaveCalibrationModel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::SaveCalibrationModel_Request & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>()
{
  return "thermal_calibration_interfaces::srv::SaveCalibrationModel_Request";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>()
{
  return "thermal_calibration_interfaces/srv/SaveCalibrationModel_Request";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveCalibrationModel_Response & msg,
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
    out << ", ";
  }

  // member: path
  {
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveCalibrationModel_Response & msg,
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

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveCalibrationModel_Response & msg, bool use_flow_style = false)
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
  const thermal_calibration_interfaces::srv::SaveCalibrationModel_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::SaveCalibrationModel_Response & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>()
{
  return "thermal_calibration_interfaces::srv::SaveCalibrationModel_Response";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>()
{
  return "thermal_calibration_interfaces/srv/SaveCalibrationModel_Response";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::SaveCalibrationModel>()
{
  return "thermal_calibration_interfaces::srv::SaveCalibrationModel";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::SaveCalibrationModel>()
{
  return "thermal_calibration_interfaces/srv/SaveCalibrationModel";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::SaveCalibrationModel>
  : std::integral_constant<
    bool,
    has_fixed_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>::value &&
    has_fixed_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>::value
  >
{
};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::SaveCalibrationModel>
  : std::integral_constant<
    bool,
    has_bounded_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>::value &&
    has_bounded_size<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>::value
  >
{
};

template<>
struct is_service<thermal_calibration_interfaces::srv::SaveCalibrationModel>
  : std::true_type
{
};

template<>
struct is_service_request<thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>
  : std::true_type
{
};

template<>
struct is_service_response<thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__TRAITS_HPP_
