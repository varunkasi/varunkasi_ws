// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CalibrationPoint in the package thermal_calibration_interfaces.
/**
  * A single thermal calibration point
 */
typedef struct thermal_calibration_interfaces__msg__CalibrationPoint
{
  /// Unique identifier for this point
  uint32_t id;
  /// X coordinate in the image
  uint16_t x;
  /// Y coordinate in the image
  uint16_t y;
  /// Raw thermal sensor value
  uint16_t raw_value;
  /// Reference temperature in Celsius
  float reference_temp;
  /// ISO timestamp when point was captured
  rosidl_runtime_c__String timestamp;
} thermal_calibration_interfaces__msg__CalibrationPoint;

// Struct for a sequence of thermal_calibration_interfaces__msg__CalibrationPoint.
typedef struct thermal_calibration_interfaces__msg__CalibrationPoint__Sequence
{
  thermal_calibration_interfaces__msg__CalibrationPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__msg__CalibrationPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_POINT__STRUCT_H_
