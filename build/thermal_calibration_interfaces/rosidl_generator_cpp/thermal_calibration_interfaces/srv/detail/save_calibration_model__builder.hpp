// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from thermal_calibration_interfaces:srv/SaveCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__BUILDER_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "thermal_calibration_interfaces/srv/detail/save_calibration_model__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_SaveCalibrationModel_Request_filename
{
public:
  Init_SaveCalibrationModel_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Request filename(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::SaveCalibrationModel_Request>()
{
  return thermal_calibration_interfaces::srv::builder::Init_SaveCalibrationModel_Request_filename();
}

}  // namespace thermal_calibration_interfaces


namespace thermal_calibration_interfaces
{

namespace srv
{

namespace builder
{

class Init_SaveCalibrationModel_Response_path
{
public:
  explicit Init_SaveCalibrationModel_Response_path(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response & msg)
  : msg_(msg)
  {}
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response path(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response msg_;
};

class Init_SaveCalibrationModel_Response_message
{
public:
  explicit Init_SaveCalibrationModel_Response_message(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response & msg)
  : msg_(msg)
  {}
  Init_SaveCalibrationModel_Response_path message(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SaveCalibrationModel_Response_path(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response msg_;
};

class Init_SaveCalibrationModel_Response_success
{
public:
  Init_SaveCalibrationModel_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveCalibrationModel_Response_message success(::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SaveCalibrationModel_Response_message(msg_);
  }

private:
  ::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::thermal_calibration_interfaces::srv::SaveCalibrationModel_Response>()
{
  return thermal_calibration_interfaces::srv::builder::Init_SaveCalibrationModel_Response_success();
}

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__SAVE_CALIBRATION_MODEL__BUILDER_HPP_
