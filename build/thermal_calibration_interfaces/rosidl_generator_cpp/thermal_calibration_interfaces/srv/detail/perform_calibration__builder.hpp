// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/PerformCalibration.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/perform_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_PerformCalibration_Request_degree
{
public:
  explicit Init_PerformCalibration_Request_degree(::thermal_calibration_interfaces::srv::PerformCalibration_Request & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::PerformCalibration_Request degree(::thermal_calibration_interfaces::srv::PerformCalibration_Request::_degree_type arg)
  {
    msg_.degree = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Request msg_;
};

class Init_PerformCalibration_Request_model_type
{
public:
  Init_PerformCalibration_Request_model_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformCalibration_Request_degree model_type(::thermal_calibration_interfaces::srv::PerformCalibration_Request::_model_type_type arg)
  {
    msg_.model_type = std::move(arg);
    return Init_PerformCalibration_Request_degree(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::PerformCalibration_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_PerformCalibration_Request_model_type();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_PerformCalibration_Response_rmse
{
public:
  explicit Init_PerformCalibration_Response_rmse(::thermal_calibration_interfaces::srv::PerformCalibration_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response rmse(::thermal_calibration_interfaces::srv::PerformCalibration_Response::_rmse_type arg)
  {
    msg_.rmse = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response msg_;
};

class Init_PerformCalibration_Response_r_squared
{
public:
  explicit Init_PerformCalibration_Response_r_squared(::thermal_calibration_interfaces::srv::PerformCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_PerformCalibration_Response_rmse r_squared(::thermal_calibration_interfaces::srv::PerformCalibration_Response::_r_squared_type arg)
  {
    msg_.r_squared = std::move(arg);
    return Init_PerformCalibration_Response_rmse(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response msg_;
};

class Init_PerformCalibration_Response_model_parameters
{
public:
  explicit Init_PerformCalibration_Response_model_parameters(::thermal_calibration_interfaces::srv::PerformCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_PerformCalibration_Response_r_squared model_parameters(::thermal_calibration_interfaces::srv::PerformCalibration_Response::_model_parameters_type arg)
  {
    msg_.model_parameters = std::move(arg);
    return Init_PerformCalibration_Response_r_squared(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response msg_;
};

class Init_PerformCalibration_Response_message
{
public:
  explicit Init_PerformCalibration_Response_message(::thermal_calibration_interfaces::srv::PerformCalibration_Response & msg)
  : msg_(msg)
  {}
  Init_PerformCalibration_Response_model_parameters message(::thermal_calibration_interfaces::srv::PerformCalibration_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_PerformCalibration_Response_model_parameters(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response msg_;
};

class Init_PerformCalibration_Response_success
{
public:
  Init_PerformCalibration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformCalibration_Response_message success(::thermal_calibration_interfaces::srv::PerformCalibration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PerformCalibration_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::PerformCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::PerformCalibration_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_PerformCalibration_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__BUILDER_HPP_
