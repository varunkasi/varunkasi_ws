// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from thermal_calibration_interfaces:srv/LoadCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__TRAITS_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "thermal_calibration_interfaces/srv/detail/load_calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadCalibrationModel_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    out << "path: ";
    rosidl_generator_traits::value_to_yaml(msg.path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadCalibrationModel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const LoadCalibrationModel_Request & msg, bool use_flow_style = false)
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
  const thermal_calibration_interfaces::srv::LoadCalibrationModel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::LoadCalibrationModel_Request & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>()
{
  return "thermal_calibration_interfaces::srv::LoadCalibrationModel_Request";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>()
{
  return "thermal_calibration_interfaces/srv/LoadCalibrationModel_Request";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace thermal_calibration_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadCalibrationModel_Response & msg,
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

  // member: model_type
  {
    out << "model_type: ";
    rosidl_generator_traits::value_to_yaml(msg.model_type, out);
    out << ", ";
  }

  // member: model_parameters
  {
    if (msg.model_parameters.size() == 0) {
      out << "model_parameters: []";
    } else {
      out << "model_parameters: [";
      size_t pending_items = msg.model_parameters.size();
      for (auto item : msg.model_parameters) {
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
  const LoadCalibrationModel_Response & msg,
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

  // member: model_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "model_type: ";
    rosidl_generator_traits::value_to_yaml(msg.model_type, out);
    out << "\n";
  }

  // member: model_parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.model_parameters.size() == 0) {
      out << "model_parameters: []\n";
    } else {
      out << "model_parameters:\n";
      for (auto item : msg.model_parameters) {
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

inline std::string to_yaml(const LoadCalibrationModel_Response & msg, bool use_flow_style = false)
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
  const thermal_calibration_interfaces::srv::LoadCalibrationModel_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  thermal_calibration_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use thermal_calibration_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const thermal_calibration_interfaces::srv::LoadCalibrationModel_Response & msg)
{
  return thermal_calibration_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>()
{
  return "thermal_calibration_interfaces::srv::LoadCalibrationModel_Response";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>()
{
  return "thermal_calibration_interfaces/srv/LoadCalibrationModel_Response";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<thermal_calibration_interfaces::srv::LoadCalibrationModel>()
{
  return "thermal_calibration_interfaces::srv::LoadCalibrationModel";
}

template<>
inline const char * name<thermal_calibration_interfaces::srv::LoadCalibrationModel>()
{
  return "thermal_calibration_interfaces/srv/LoadCalibrationModel";
}

template<>
struct has_fixed_size<thermal_calibration_interfaces::srv::LoadCalibrationModel>
  : std::integral_constant<
    bool,
    has_fixed_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>::value &&
    has_fixed_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>::value
  >
{
};

template<>
struct has_bounded_size<thermal_calibration_interfaces::srv::LoadCalibrationModel>
  : std::integral_constant<
    bool,
    has_bounded_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>::value &&
    has_bounded_size<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>::value
  >
{
};

template<>
struct is_service<thermal_calibration_interfaces::srv::LoadCalibrationModel>
  : std::true_type
{
};

template<>
struct is_service_request<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>
  : std::true_type
{
};

template<>
struct is_service_response<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__TRAITS_HPP_
