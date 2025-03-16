// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from thermal_calibration_interfaces:srv/LoadCalibrationModel.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_H_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadCalibrationModel in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__LoadCalibrationModel_Request
{
  /// Path to calibration model file
  rosidl_runtime_c__String path;
} thermal_calibration_interfaces__srv__LoadCalibrationModel_Request;

// Struct for a sequence of thermal_calibration_interfaces__srv__LoadCalibrationModel_Request.
typedef struct thermal_calibration_interfaces__srv__LoadCalibrationModel_Request__Sequence
{
  thermal_calibration_interfaces__srv__LoadCalibrationModel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__LoadCalibrationModel_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'model_type'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'model_parameters'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/LoadCalibrationModel in the package thermal_calibration_interfaces.
typedef struct thermal_calibration_interfaces__srv__LoadCalibrationModel_Response
{
  /// Whether the operation was successful
  bool success;
  /// Status message
  rosidl_runtime_c__String message;
  /// Type of loaded model
  rosidl_runtime_c__String model_type;
  /// Loaded model parameters
  rosidl_runtime_c__float__Sequence model_parameters;
} thermal_calibration_interfaces__srv__LoadCalibrationModel_Response;

// Struct for a sequence of thermal_calibration_interfaces__srv__LoadCalibrationModel_Response.
typedef struct thermal_calibration_interfaces__srv__LoadCalibrationModel_Response__Sequence
{
  thermal_calibration_interfaces__srv__LoadCalibrationModel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} thermal_calibration_interfaces__srv__LoadCalibrationModel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__LOAD_CALIBRATION_MODEL__STRUCT_H_
