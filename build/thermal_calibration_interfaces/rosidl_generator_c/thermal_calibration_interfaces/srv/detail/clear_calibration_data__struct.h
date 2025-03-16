// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:srv/ClearCalibrationData.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ClearCalibrationData in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__ClearCalibrationData_Request
{
  /// Confirmation flag to avoid accidental clearing
  bool confirm;
} thermal_calibration_interfaces__srv__ClearCalibrationData_Request;

// Struct for a sequence of thermal_calibration_interfaces__srv__ClearCalibrationData_Request.
typedef struct thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence
{
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ClearCalibrationData in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__ClearCalibrationData_Response
{
  /// Whether the operation was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
} thermal_calibration_interfaces__srv__ClearCalibrationData_Response;

// Struct for a sequence of thermal_calibration_interfaces__srv__ClearCalibrationData_Response.
typedef struct thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence
{
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__CLEAR_CALIBRATION_DATA__STRUCT_H_
