// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:srv/LoadCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Request __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Request __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoadCalibrationModel_Request_
{
  using Type = LoadCalibrationModel_Request_<ContainerAllocator>;

  explicit LoadCalibrationModel_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
    }
  }

  explicit LoadCalibrationModel_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
    }
  }

  // field types and members
  using _path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoadCalibrationModel_Request_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoadCalibrationModel_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoadCalibrationModel_Request_

// alias to use template instance with default allocator
using LoadCalibrationModel_Request =
  thermal_calibration_interfaces::srv::LoadCalibrationModel_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Response __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Response __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LoadCalibrationModel_Response_
{
  using Type = LoadCalibrationModel_Response_<ContainerAllocator>;

  explicit LoadCalibrationModel_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->model_type = "";
    }
  }

  explicit LoadCalibrationModel_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    model_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->model_type = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _model_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _model_type_type model_type;
  using _model_parameters_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _model_parameters_type model_parameters;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__model_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->model_type = _arg;
    return *this;
  }
  Type & set__model_parameters(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->model_parameters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__LoadCalibrationModel_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoadCalibrationModel_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->model_type != other.model_type) {
      return false;
    }
    if (this->model_parameters != other.model_parameters) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoadCalibrationModel_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoadCalibrationModel_Response_

// alias to use template instance with default allocator
using LoadCalibrationModel_Response =
  thermal_calibration_interfaces::srv::LoadCalibrationModel_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace thermal_calibration_interfaces
{

namespace srv
{

struct LoadCalibrationModel
{
  using Request = thermal_calibration_interfaces::srv::LoadCalibrationModel_Request;
  using Response = thermal_calibration_interfaces::srv::LoadCalibrationModel_Response;
};

}  // namespace srv

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_HPP_
