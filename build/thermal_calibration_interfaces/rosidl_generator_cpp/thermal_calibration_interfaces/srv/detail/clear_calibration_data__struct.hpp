// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:srv/ClearCalibrationData.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Request __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Request __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClearCalibrationData_Request_
{
  using Type = ClearCalibrationData_Request_<ContainerAllocator>;

  explicit ClearCalibrationData_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirm = false;
    }
  }

  explicit ClearCalibrationData_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirm = false;
    }
  }

  // field types and members
  using _confirm_type =
    bool;
  _confirm_type confirm;

  // setters for named parameter idiom
  Type & set__confirm(
    const bool & _arg)
  {
    this->confirm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClearCalibrationData_Request_ & other) const
  {
    if (this->confirm != other.confirm) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClearCalibrationData_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClearCalibrationData_Request_

// alias to use template instance with default allocator
using ClearCalibrationData_Request =
  thermal_calibration_interfaces::srv::ClearCalibrationData_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Response __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Response __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClearCalibrationData_Response_
{
  using Type = ClearCalibrationData_Response_<ContainerAllocator>;

  explicit ClearCalibrationData_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ClearCalibrationData_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

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

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__ClearCalibrationData_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClearCalibrationData_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClearCalibrationData_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClearCalibrationData_Response_

// alias to use template instance with default allocator
using ClearCalibrationData_Response =
  thermal_calibration_interfaces::srv::ClearCalibrationData_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace thermal_calibration_interfaces
{

namespace srv
{

struct ClearCalibrationData
{
  using Request = thermal_calibration_interfaces::srv::ClearCalibrationData_Request;
  using Response = thermal_calibration_interfaces::srv::ClearCalibrationData_Response;
};

}  // namespace srv

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_HPP_
