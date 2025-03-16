// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__msg__CalibrationModel __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__msg__CalibrationModel __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CalibrationModel_
{
  using Type = CalibrationModel_<ContainerAllocator>;

  explicit CalibrationModel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->model_type = "";
      this->degree = 0;
      this->r_squared = 0.0f;
      this->rmse = 0.0f;
      this->points_count = 0ul;
    }
  }

  explicit CalibrationModel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : model_type(_alloc),
    timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->model_type = "";
      this->degree = 0;
      this->r_squared = 0.0f;
      this->rmse = 0.0f;
      this->points_count = 0ul;
    }
  }

  // field types and members
  using _model_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _model_type_type model_type;
  using _degree_type =
    int8_t;
  _degree_type degree;
  using _parameters_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _parameters_type parameters;
  using _r_squared_type =
    float;
  _r_squared_type r_squared;
  using _rmse_type =
    float;
  _rmse_type rmse;
  using _points_count_type =
    uint32_t;
  _points_count_type points_count;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _raw_value_range_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _raw_value_range_type raw_value_range;

  // setters for named parameter idiom
  Type & set__model_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->model_type = _arg;
    return *this;
  }
  Type & set__degree(
    const int8_t & _arg)
  {
    this->degree = _arg;
    return *this;
  }
  Type & set__parameters(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }
  Type & set__r_squared(
    const float & _arg)
  {
    this->r_squared = _arg;
    return *this;
  }
  Type & set__rmse(
    const float & _arg)
  {
    this->rmse = _arg;
    return *this;
  }
  Type & set__points_count(
    const uint32_t & _arg)
  {
    this->points_count = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__raw_value_range(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->raw_value_range = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__msg__CalibrationModel
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__msg__CalibrationModel
    std::shared_ptr<thermal_calibration_interfaces::msg::CalibrationModel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrationModel_ & other) const
  {
    if (this->model_type != other.model_type) {
      return false;
    }
    if (this->degree != other.degree) {
      return false;
    }
    if (this->parameters != other.parameters) {
      return false;
    }
    if (this->r_squared != other.r_squared) {
      return false;
    }
    if (this->rmse != other.rmse) {
      return false;
    }
    if (this->points_count != other.points_count) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->raw_value_range != other.raw_value_range) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrationModel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrationModel_

// alias to use template instance with default allocator
using CalibrationModel =
  thermal_calibration_interfaces::msg::CalibrationModel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_HPP_
