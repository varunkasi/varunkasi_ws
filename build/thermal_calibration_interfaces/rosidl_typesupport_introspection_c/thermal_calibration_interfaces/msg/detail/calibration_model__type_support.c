// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "thermal_calibration_interfaces/msg/detail/calibration_model__rosidl_typesupport_introspection_c.h"
#include "thermal_calibration_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "thermal_calibration_interfaces/msg/detail/calibration_model__functions.h"
#include "thermal_calibration_interfaces/msg/detail/calibration_model__struct.h"


// Include directives for member types
// Member `model_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `parameters`
// Member `raw_value_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  thermal_calibration_interfaces__msg__CalibrationModel__init(message_memory);
}

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_fini_function(void * message_memory)
{
  thermal_calibration_interfaces__msg__CalibrationModel__fini(message_memory);
}

size_t thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__size_function__CalibrationModel__parameters(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__parameters(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__parameters(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__fetch_function__CalibrationModel__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__parameters(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__assign_function__CalibrationModel__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__parameters(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__resize_function__CalibrationModel__parameters(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__size_function__CalibrationModel__raw_value_range(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__raw_value_range(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__raw_value_range(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__fetch_function__CalibrationModel__raw_value_range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__raw_value_range(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__assign_function__CalibrationModel__raw_value_range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__raw_value_range(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__resize_function__CalibrationModel__raw_value_range(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_member_array[8] = {
  {
    "model_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, model_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "degree",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, degree),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parameters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, parameters),  // bytes offset in struct
    NULL,  // default value
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__size_function__CalibrationModel__parameters,  // size() function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__parameters,  // get_const(index) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__parameters,  // get(index) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__fetch_function__CalibrationModel__parameters,  // fetch(index, &value) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__assign_function__CalibrationModel__parameters,  // assign(index, value) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__resize_function__CalibrationModel__parameters  // resize(index) function pointer
  },
  {
    "r_squared",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, r_squared),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rmse",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, rmse),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, points_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "raw_value_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationModel, raw_value_range),  // bytes offset in struct
    NULL,  // default value
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__size_function__CalibrationModel__raw_value_range,  // size() function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_const_function__CalibrationModel__raw_value_range,  // get_const(index) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__get_function__CalibrationModel__raw_value_range,  // get(index) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__fetch_function__CalibrationModel__raw_value_range,  // fetch(index, &value) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__assign_function__CalibrationModel__raw_value_range,  // assign(index, value) function pointer
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__resize_function__CalibrationModel__raw_value_range  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_members = {
  "thermal_calibration_interfaces__msg",  // message namespace
  "CalibrationModel",  // message name
  8,  // number of fields
  sizeof(thermal_calibration_interfaces__msg__CalibrationModel),
  thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_member_array,  // message members
  thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_init_function,  // function to initialize message memory (memory has to be allocated)
  thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_type_support_handle = {
  0,
  &thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_thermal_calibration_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, thermal_calibration_interfaces, msg, CalibrationModel)() {
  thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_type_support_handle.typesupport_identifier) {
    thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &thermal_calibration_interfaces__msg__CalibrationModel__rosidl_typesupport_introspection_c__CalibrationModel_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
