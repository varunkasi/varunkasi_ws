// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:srv/PerformCalibration.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__STRUCT_H_

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

/// Struct defined in srv/PerformCalibration in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__PerformCalibration_Request
{
  /// Type of model to use (e.g., "polynomial")
  rosidl_runtime_c__String model_type;
  /// Degree of polynomial (if applicable)
  int8_t degree;
} thermal_calibration_interfaces__srv__PerformCalibration_Request;

// Struct for a sequence of thermal_calibration_interfaces__srv__PerformCalibration_Request.
typedef struct thermal_calibration_interfaces__srv__PerformCalibration_Request__Sequence
{
  thermal_calibration_interfaces__srv__PerformCalibration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__PerformCalibration_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'model_parameters'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/PerformCalibration in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__PerformCalibration_Response
{
  /// Whether the operation was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Calibration model parameters
  rosidl_runtime_c__float__Sequence model_parameters;
  /// R-squared value of the calibration
  float r_squared;
  /// Root mean square error in Celsius
  float rmse;
} thermal_calibration_interfaces__srv__PerformCalibration_Response;

// Struct for a sequence of thermal_calibration_interfaces__srv__PerformCalibration_Response.
typedef struct thermal_calibration_interfaces__srv__PerformCalibration_Response__Sequence
{
  thermal_calibration_interfaces__srv__PerformCalibration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__PerformCalibration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__PERFORM_CALIBRATION__STRUCT_H_
