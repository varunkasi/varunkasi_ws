// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__msg__CalibrationPoint __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__msg__CalibrationPoint __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CalibrationPoint_
{
  using Type = CalibrationPoint_<ContainerAllocator>;

  explicit CalibrationPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->x = 0;
      this->y = 0;
      this->raw_value = 0;
      this->reference_temp = 0.0f;
      this->timestamp = "";
    }
  }

  explicit CalibrationPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->x = 0;
      this->y = 0;
      this->raw_value = 0;
      this->reference_temp = 0.0f;
      this->timestamp = "";
    }
  }

  // field types and members
  using _id_type =
    uint32_t;
  _id_type id;
  using _x_type =
    uint16_t;
  _x_type x;
  using _y_type =
    uint16_t;
  _y_type y;
  using _raw_value_type =
    uint16_t;
  _raw_value_type raw_value;
  using _reference_temp_type =
    float;
  _reference_temp_type reference_temp;
  using _timestamp_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__x(
    const uint16_t & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const uint16_t & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__raw_value(
    const uint16_t & _arg)
  {
    this->raw_value = _arg;
    return *this;
  }
  Type & set__reference_temp(
    const float & _arg)
  {
    this->reference_temp = _arg;
    return *this;
  }
  Type & set__timestamp(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__msg__CalibrationPoint
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__msg__CalibrationPoint
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrationPoint_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->raw_value != other.raw_value) {
      return false;
    }
    if (this->reference_temp != other.reference_temp) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrationPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrationPoint_

// alias to use template instance with default allocator
using CalibrationPoint =
  thermal_calibration_interfaces::msg::CalibrationPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_HPP_
