// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from thermal_calibration_interfaces:srv/AddCalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__STRUCT_HPP_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Request __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AddCalibrationPoint_Request_
{
  using Type = AddCalibrationPoint_Request_<ContainerAllocator>;

  explicit AddCalibrationPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0;
      this->y = 0;
      this->raw_value = 0;
      this->reference_temp = 0.0f;
    }
  }

  explicit AddCalibrationPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0;
      this->y = 0;
      this->raw_value = 0;
      this->reference_temp = 0.0f;
    }
  }

  // field types and members
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

  // setters for named parameter idiom
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

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Request
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AddCalibrationPoint_Request_ & other) const
  {
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
    return true;
  }
  bool operator!=(const AddCalibrationPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AddCalibrationPoint_Request_

// alias to use template instance with default allocator
using AddCalibrationPoint_Request =
  thermal_calibration_interfaces::srv::AddCalibrationPoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces


#ifndef _WIN32
# define DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Response __declspec(deprecated)
#endif

namespace thermal_calibration_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AddCalibrationPoint_Response_
{
  using Type = AddCalibrationPoint_Response_<ContainerAllocator>;

  explicit AddCalibrationPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->point_id = 0ul;
    }
  }

  explicit AddCalibrationPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->point_id = 0ul;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _point_id_type =
    uint32_t;
  _point_id_type point_id;

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
  Type & set__point_id(
    const uint32_t & _arg)
  {
    this->point_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__thermal_calibration_interfaces__srv__AddCalibrationPoint_Response
    std::shared_ptr<thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AddCalibrationPoint_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->point_id != other.point_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const AddCalibrationPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AddCalibrationPoint_Response_

// alias to use template instance with default allocator
using AddCalibrationPoint_Response =
  thermal_calibration_interfaces::srv::AddCalibrationPoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace thermal_calibration_interfaces

namespace thermal_calibration_interfaces
{

namespace srv
{

struct AddCalibrationPoint
{
  using Request = thermal_calibration_interfaces::srv::AddCalibrationPoint_Request;
  using Response = thermal_calibration_interfaces::srv::AddCalibrationPoint_Response;
};

}  // namespace srv

}  // namespace thermal_calibration_interfaces

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__STRUCT_HPP_
