// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/msg/detail/calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace msg
{

namespace builder
{

class Init_CalibrationModel_raw_value_range
{
public:
  explicit Init_CalibrationModel_raw_value_range(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::msg::CalibrationModel raw_value_range(::thermal_calibration_interfaces::msg::CalibrationModel::_raw_value_range_type arg)
  {
    msg_.raw_value_range = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_timestamp
{
public:
  explicit Init_CalibrationModel_timestamp(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_raw_value_range timestamp(::thermal_calibration_interfaces::msg::CalibrationModel::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_CalibrationModel_raw_value_range(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_points_count
{
public:
  explicit Init_CalibrationModel_points_count(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_timestamp points_count(::thermal_calibration_interfaces::msg::CalibrationModel::_points_count_type arg)
  {
    msg_.points_count = std::move(arg);
    return Init_CalibrationModel_timestamp(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_rmse
{
public:
  explicit Init_CalibrationModel_rmse(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_points_count rmse(::thermal_calibration_interfaces::msg::CalibrationModel::_rmse_type arg)
  {
    msg_.rmse = std::move(arg);
    return Init_CalibrationModel_points_count(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_r_squared
{
public:
  explicit Init_CalibrationModel_r_squared(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_rmse r_squared(::thermal_calibration_interfaces::msg::CalibrationModel::_r_squared_type arg)
  {
    msg_.r_squared = std::move(arg);
    return Init_CalibrationModel_rmse(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_parameters
{
public:
  explicit Init_CalibrationModel_parameters(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_r_squared parameters(::thermal_calibration_interfaces::msg::CalibrationModel::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return Init_CalibrationModel_r_squared(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_degree
{
public:
  explicit Init_CalibrationModel_degree(::thermal_calibration_interfaces::msg::CalibrationModel & msg)
  : msg_(msg)
  {}
  Init_CalibrationModel_parameters degree(::thermal_calibration_interfaces::msg::CalibrationModel::_degree_type arg)
  {
    msg_.degree = std::move(arg);
    return Init_CalibrationModel_parameters(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

class Init_CalibrationModel_model_type
{
public:
  Init_CalibrationModel_model_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrationModel_degree model_type(::thermal_calibration_interfaces::msg::CalibrationModel::_model_type_type arg)
  {
    msg_.model_type = std::move(arg);
    return Init_CalibrationModel_degree(msg_);
  }

private:
  ::thermal_calibration_interfaces::msg::CalibrationModel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::msg::CalibrationModel>()
{
  return thermal_calibration_interfaces::msg::builder::Init_CalibrationModel_model_type();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__BUILDER_HPP_
