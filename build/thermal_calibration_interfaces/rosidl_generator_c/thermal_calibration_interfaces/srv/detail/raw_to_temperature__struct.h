// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:srv/RawToTemperature.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RawToTemperature in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__RawToTemperature_Request
{
  /// Raw thermal sensor value
  uint16_t raw_value;
} thermal_calibration_interfaces__srv__RawToTemperature_Request;

// Struct for a sequence of thermal_calibration_interfaces__srv__RawToTemperature_Request.
typedef struct thermal_calibration_interfaces__srv__RawToTemperature_Request__Sequence
{
  thermal_calibration_interfaces__srv__RawToTemperature_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__RawToTemperature_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RawToTemperature in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__RawToTemperature_Response
{
  /// Whether the operation was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Estimated temperature in Celsius
  float temperature;
} thermal_calibration_interfaces__srv__RawToTemperature_Response;

// Struct for a sequence of thermal_calibration_interfaces__srv__RawToTemperature_Response.
typedef struct thermal_calibration_interfaces__srv__RawToTemperature_Response__Sequence
{
  thermal_calibration_interfaces__srv__RawToTemperature_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__RawToTemperature_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__RAW_TO_TEMPERATURE__STRUCT_H_
