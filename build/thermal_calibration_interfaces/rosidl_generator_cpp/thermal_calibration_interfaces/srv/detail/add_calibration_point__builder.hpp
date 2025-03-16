// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/AddCalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/add_calibration_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_AddCalibrationPoint_Request_reference_temp
{
public:
  explicit Init_AddCalibrationPoint_Request_reference_temp(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request reference_temp(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request::_reference_temp_type arg)
  {
    msg_.reference_temp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request msg_;
};

class Init_AddCalibrationPoint_Request_raw_value
{
public:
  explicit Init_AddCalibrationPoint_Request_raw_value(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request & msg)
  : msg_(msg)
  {}
  Init_AddCalibrationPoint_Request_reference_temp raw_value(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request::_raw_value_type arg)
  {
    msg_.raw_value = std::move(arg);
    return Init_AddCalibrationPoint_Request_reference_temp(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request msg_;
};

class Init_AddCalibrationPoint_Request_y
{
public:
  explicit Init_AddCalibrationPoint_Request_y(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request & msg)
  : msg_(msg)
  {}
  Init_AddCalibrationPoint_Request_raw_value y(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_AddCalibrationPoint_Request_raw_value(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request msg_;
};

class Init_AddCalibrationPoint_Request_x
{
public:
  Init_AddCalibrationPoint_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddCalibrationPoint_Request_y x(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_AddCalibrationPoint_Request_y(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::AddCalibrationPoint_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_AddCalibrationPoint_Request_x();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_AddCalibrationPoint_Response_point_id
{
public:
  explicit Init_AddCalibrationPoint_Response_point_id(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response point_id(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response::_point_id_type arg)
  {
    msg_.point_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response msg_;
};

class Init_AddCalibrationPoint_Response_message
{
public:
  explicit Init_AddCalibrationPoint_Response_message(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response & msg)
  : msg_(msg)
  {}
  Init_AddCalibrationPoint_Response_point_id message(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_AddCalibrationPoint_Response_point_id(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response msg_;
};

class Init_AddCalibrationPoint_Response_success
{
public:
  Init_AddCalibrationPoint_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddCalibrationPoint_Response_message success(::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_AddCalibrationPoint_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::AddCalibrationPoint_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_AddCalibrationPoint_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__BUILDER_HPP_
