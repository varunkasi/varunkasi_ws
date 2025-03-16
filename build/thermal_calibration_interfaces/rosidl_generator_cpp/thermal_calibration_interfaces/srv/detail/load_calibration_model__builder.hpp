// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/LoadCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/load_calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_LoadCalibrationModel_Request_path
{
public:
  Init_LoadCalibrationModel_Request_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Request path(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::LoadCalibrationModel_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_LoadCalibrationModel_Request_path();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_LoadCalibrationModel_Response_model_parameters
{
public:
  explicit Init_LoadCalibrationModel_Response_model_parameters(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response model_parameters(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response::_model_parameters_type arg)
  {
    msg_.model_parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response msg_;
};

class Init_LoadCalibrationModel_Response_model_type
{
public:
  explicit Init_LoadCalibrationModel_Response_model_type(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response & msg)
  : msg_(msg)
  {}
  Init_LoadCalibrationModel_Response_model_parameters model_type(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response::_model_type_type arg)
  {
    msg_.model_type = std::move(arg);
    return Init_LoadCalibrationModel_Response_model_parameters(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response msg_;
};

class Init_LoadCalibrationModel_Response_message
{
public:
  explicit Init_LoadCalibrationModel_Response_message(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response & msg)
  : msg_(msg)
  {}
  Init_LoadCalibrationModel_Response_model_type message(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_LoadCalibrationModel_Response_model_type(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response msg_;
};

class Init_LoadCalibrationModel_Response_success
{
public:
  Init_LoadCalibrationModel_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoadCalibrationModel_Response_message success(::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LoadCalibrationModel_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::LoadCalibrationModel_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_LoadCalibrationModel_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__BUILDER_HPP_
