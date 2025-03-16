// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "thermal_calibration_interfaces/msg/detail/calibration_point__rosidl_typesupport_introspection_c.h"
#include "thermal_calibration_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "thermal_calibration_interfaces/msg/detail/calibration_point__functions.h"
#include "thermal_calibration_interfaces/msg/detail/calibration_point__struct.h"


// Include directives for member types
// Member `timestamp`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  thermal_calibration_interfaces__msg__CalibrationPoint__init(message_memory);
}

void thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_fini_function(void * message_memory)
{
  thermal_calibration_interfaces__msg__CalibrationPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_member_array[6] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "raw_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, raw_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference_temp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, reference_temp),  // bytes offset in struct
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
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(thermal_calibration_interfaces__msg__CalibrationPoint, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_members = {
  "thermal_calibration_interfaces__msg",  // message namespace
  "CalibrationPoint",  // message name
  6,  // number of fields
  sizeof(thermal_calibration_interfaces__msg__CalibrationPoint),
  thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_member_array,  // message members
  thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_type_support_handle = {
  0,
  &thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_thermal_calibration_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, thermal_calibration_interfaces, msg, CalibrationPoint)() {
  if (!thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_type_support_handle.typesupport_identifier) {
    thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &thermal_calibration_interfaces__msg__CalibrationPoint__rosidl_typesupport_introspection_c__CalibrationPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
