// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/msg/detail/calibration_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace msg
{

namespace builder
{

class Init_CalibrationPoint_timestamp
{
public:
  explicit Init_CalibrationPoint_timestamp(::thermal_calibration_interfaces::msg::CalibrationPoint & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::msg::CalibrationPoint timestamp(::thermal_calibration_interfaces::msg::CalibrationPoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

class Init_CalibrationPoint_reference_temp
{
public:
  explicit Init_CalibrationPoint_reference_temp(::thermal_calibration_interfaces::msg::CalibrationPoint & msg)
  : msg_(msg)
  {}
  Init_CalibrationPoint_timestamp reference_temp(::thermal_calibration_interfaces::msg::CalibrationPoint::_reference_temp_type arg)
  {
    msg_.reference_temp = std::move(arg);
    return Init_CalibrationPoint_timestamp(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

class Init_CalibrationPoint_raw_value
{
public:
  explicit Init_CalibrationPoint_raw_value(::thermal_calibration_interfaces::msg::CalibrationPoint & msg)
  : msg_(msg)
  {}
  Init_CalibrationPoint_reference_temp raw_value(::thermal_calibration_interfaces::msg::CalibrationPoint::_raw_value_type arg)
  {
    msg_.raw_value = std::move(arg);
    return Init_CalibrationPoint_reference_temp(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

class Init_CalibrationPoint_y
{
public:
  explicit Init_CalibrationPoint_y(::thermal_calibration_interfaces::msg::CalibrationPoint & msg)
  : msg_(msg)
  {}
  Init_CalibrationPoint_raw_value y(::thermal_calibration_interfaces::msg::CalibrationPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CalibrationPoint_raw_value(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

class Init_CalibrationPoint_x
{
public:
  explicit Init_CalibrationPoint_x(::thermal_calibration_interfaces::msg::CalibrationPoint & msg)
  : msg_(msg)
  {}
  Init_CalibrationPoint_y x(::thermal_calibration_interfaces::msg::CalibrationPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CalibrationPoint_y(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

class Init_CalibrationPoint_id
{
public:
  Init_CalibrationPoint_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrationPoint_x id(::thermal_calibration_interfaces::msg::CalibrationPoint::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_CalibrationPoint_x(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::msg::CalibrationPoint>()
{
  return thermal_calibration_interfaces::msg::builder::Init_CalibrationPoint_id();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__BUILDER_HPP_
