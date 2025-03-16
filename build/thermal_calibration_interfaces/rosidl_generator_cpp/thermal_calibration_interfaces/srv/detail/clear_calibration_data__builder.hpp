// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/ClearCalibrationData.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/clear_calibration_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_ClearCalibrationData_Request_confirm
{
public:
  Init_ClearCalibrationData_Request_confirm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::thermal_calibration_interfaces::srv::ClearCalibrationData_Request confirm(::thermal_calibration_interfaces::srv::ClearCalibrationData_Request::_confirm_type arg)
  {
    msg_.confirm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::ClearCalibrationData_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::ClearCalibrationData_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_ClearCalibrationData_Request_confirm();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_ClearCalibrationData_Response_message
{
public:
  explicit Init_ClearCalibrationData_Response_message(::thermal_calibration_interfaces::srv::ClearCalibrationData_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::ClearCalibrationData_Response message(::thermal_calibration_interfaces::srv::ClearCalibrationData_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::ClearCalibrationData_Response msg_;
};

class Init_ClearCalibrationData_Response_success
{
public:
  Init_ClearCalibrationData_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ClearCalibrationData_Response_message success(::thermal_calibration_interfaces::srv::ClearCalibrationData_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ClearCalibrationData_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::ClearCalibrationData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::ClearCalibrationData_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_ClearCalibrationData_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__BUILDER_HPP_
