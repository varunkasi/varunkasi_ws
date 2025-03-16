// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:srv/RawToTemperature.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Request __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Request __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RawToTemperature_Request_
{
  using Type = RawToTemperature_Request_<ContainerAllocator>;

  explicit RawToTemperature_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_value = 0;
    }
  }

  explicit RawToTemperature_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->raw_value = 0;
    }
  }

  // field types and members
  using _raw_value_type =
    uint16_t;
  _raw_value_type raw_value;

  // setters for named parameter idiom
  Type & set__raw_value(
    const uint16_t & _arg)
  {
    this->raw_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawToTemperature_Request_ & other) const
  {
    if (this->raw_value != other.raw_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawToTemperature_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawToTemperature_Request_

// alias to use template instance with default allocator
using RawToTemperature_Request =
  thermal_calibration_interfaces::srv::RawToTemperature_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Response __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Response __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RawToTemperature_Response_
{
  using Type = RawToTemperature_Response_<ContainerAllocator>;

  explicit RawToTemperature_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->temperature = 0.0f;
    }
  }

  explicit RawToTemperature_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->temperature = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _temperature_type =
    float;
  _temperature_type temperature;

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
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__RawToTemperature_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::RawToTemperature_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawToTemperature_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawToTemperature_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawToTemperature_Response_

// alias to use template instance with default allocator
using RawToTemperature_Response =
  thermal_calibration_interfaces::srv::RawToTemperature_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace thermal_calibration_interfaces
{

namespace srv
{

struct RawToTemperature
{
  using Request = thermal_calibration_interfaces::srv::RawToTemperature_Request;
  using Response = thermal_calibration_interfaces::srv::RawToTemperature_Response;
};

}  // namespace srv

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_HPP_
