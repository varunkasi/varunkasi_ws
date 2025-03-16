// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/GetRawValue.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__GET_RAW_VALUE__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__GET_RAW_VALUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/get_raw_value__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetRawValue_Request_y
{
public:
  explicit Init_GetRawValue_Request_y(::thermal_calibration_interfaces::srv::GetRawValue_Request & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::GetRawValue_Request y(::thermal_calibration_interfaces::srv::GetRawValue_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::GetRawValue_Request msg_;
};

class Init_GetRawValue_Request_x
{
public:
  Init_GetRawValue_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRawValue_Request_y x(::thermal_calibration_interfaces::srv::GetRawValue_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GetRawValue_Request_y(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::GetRawValue_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::GetRawValue_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_GetRawValue_Request_x();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetRawValue_Response_raw_value
{
public:
  explicit Init_GetRawValue_Response_raw_value(::thermal_calibration_interfaces::srv::GetRawValue_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::GetRawValue_Response raw_value(::thermal_calibration_interfaces::srv::GetRawValue_Response::_raw_value_type arg)
  {
    msg_.raw_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::GetRawValue_Response msg_;
};

class Init_GetRawValue_Response_message
{
public:
  explicit Init_GetRawValue_Response_message(::thermal_calibration_interfaces::srv::GetRawValue_Response & msg)
  : msg_(msg)
  {}
  Init_GetRawValue_Response_raw_value message(::thermal_calibration_interfaces::srv::GetRawValue_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetRawValue_Response_raw_value(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::GetRawValue_Response msg_;
};

class Init_GetRawValue_Response_success
{
public:
  Init_GetRawValue_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRawValue_Response_message success(::thermal_calibration_interfaces::srv::GetRawValue_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetRawValue_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::GetRawValue_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::GetRawValue_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_GetRawValue_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__GET_RAW_VALUE__BUILDER_HPP_
