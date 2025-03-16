// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'model_type'
#include "rosidl_runtime_c/string.h"
// Member 'parameters'
// Member 'raw_value_range'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/CalibrationModel in the package thermal_calibration_interfaces.
/**
  * Thermal calibration model
 */
typedef struct thermal_calibration_interfaces__msg__CalibrationModel
{
  /// Type of model (e.g., "polynomial")
  rosidl_runtime_c__String model_type;
  /// Degree of polynomial (if applicable)
  int8_t degree;
  /// Model parameters/coefficients
  rosidl_runtime_c__float__Sequence parameters;
  /// R-squared value (coefficient of determination)
  float r_squared;
  /// Root mean square error (in Celsius)
  float rmse;
  /// Number of calibration points used
  uint32_t points_count;
  /// When calibration was performed
  builtin_interfaces__msg__Time timestamp;
  /// Min and max raw values covered [min, max]
  rosidl_runtime_c__uint16__Sequence raw_value_range;
} thermal_calibration_interfaces__msg__CalibrationModel;

// Struct for a sequence of thermal_calibration_interfaces__msg__CalibrationModel.
typedef struct thermal_calibration_interfaces__msg__CalibrationModel__Sequence
{
  thermal_calibration_interfaces__msg__CalibrationModel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__msg__CalibrationModel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__MSG__DETAIL__CALIBRATION_MODEL__STRUCT_H_
