// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/RawToTemperature.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/raw_to_temperature__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_RawToTemperature_Request_raw_value
{
public:
  Init_RawToTemperature_Request_raw_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::thermal_calibration_interfaces::srv::RawToTemperature_Request raw_value(::thermal_calibration_interfaces::srv::RawToTemperature_Request::_raw_value_type arg)
  {
    msg_.raw_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::RawToTemperature_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::RawToTemperature_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_RawToTemperature_Request_raw_value();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_RawToTemperature_Response_temperature
{
public:
  explicit Init_RawToTemperature_Response_temperature(::thermal_calibration_interfaces::srv::RawToTemperature_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::RawToTemperature_Response temperature(::thermal_calibration_interfaces::srv::RawToTemperature_Response::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::RawToTemperature_Response msg_;
};

class Init_RawToTemperature_Response_message
{
public:
  explicit Init_RawToTemperature_Response_message(::thermal_calibration_interfaces::srv::RawToTemperature_Response & msg)
  : msg_(msg)
  {}
  Init_RawToTemperature_Response_temperature message(::thermal_calibration_interfaces::srv::RawToTemperature_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_RawToTemperature_Response_temperature(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::RawToTemperature_Response msg_;
};

class Init_RawToTemperature_Response_success
{
public:
  Init_RawToTemperature_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RawToTemperature_Response_message success(::thermal_calibration_interfaces::srv::RawToTemperature_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RawToTemperature_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::RawToTemperature_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::RawToTemperature_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_RawToTemperature_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__BUILDER_HPP_
