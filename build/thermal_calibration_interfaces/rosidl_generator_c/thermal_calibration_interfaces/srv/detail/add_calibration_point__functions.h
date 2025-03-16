// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from thermal_calibration_interfaces:srv/AddCalibrationPoint.idl
// generated code does not contain a copyright notice

#ifndef THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__FUNCTIONS_H_
#define THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "thermal_calibration_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "thermal_calibration_interfaces/srv/detail/add_calibration_point__struct.h"

/// Initialize srv/AddCalibrationPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request
 * )) before or use
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__init(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * msg);

/// Finalize srv/AddCalibrationPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__fini(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * msg);

/// Create srv/AddCalibrationPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request *
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__create();

/// Destroy srv/AddCalibrationPoint message.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__destroy(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * msg);

/// Check for srv/AddCalibrationPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__are_equal(const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * lhs, const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * rhs);

/// Copy a srv/AddCalibrationPoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__copy(
  const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * input,
  thermal_calibration_interfaces__srv__AddCalibrationPoint_Request * output);

/// Initialize array of srv/AddCalibrationPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__init(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * array, size_t size);

/// Finalize array of srv/AddCalibrationPoint messages.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__fini(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * array);

/// Create array of srv/AddCalibrationPoint messages.
/**
 * It allocates the memory for the array and calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence *
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__create(size_t size);

/// Destroy array of srv/AddCalibrationPoint messages.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__destroy(thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * array);

/// Check for srv/AddCalibrationPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__are_equal(const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * lhs, const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * rhs);

/// Copy an array of srv/AddCalibrationPoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence__copy(
  const thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * input,
  thermal_calibration_interfaces__srv__AddCalibrationPoint_Request__Sequence * output);

/// Initialize srv/AddCalibrationPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response
 * )) before or use
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__init(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * msg);

/// Finalize srv/AddCalibrationPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__fini(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * msg);

/// Create srv/AddCalibrationPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response *
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__create();

/// Destroy srv/AddCalibrationPoint message.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__destroy(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * msg);

/// Check for srv/AddCalibrationPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__are_equal(const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * lhs, const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * rhs);

/// Copy a srv/AddCalibrationPoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__copy(
  const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * input,
  thermal_calibration_interfaces__srv__AddCalibrationPoint_Response * output);

/// Initialize array of srv/AddCalibrationPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__init(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * array, size_t size);

/// Finalize array of srv/AddCalibrationPoint messages.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__fini(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * array);

/// Create array of srv/AddCalibrationPoint messages.
/**
 * It allocates the memory for the array and calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence *
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__create(size_t size);

/// Destroy array of srv/AddCalibrationPoint messages.
/**
 * It calls
 * thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
void
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__destroy(thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * array);

/// Check for srv/AddCalibrationPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__are_equal(const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * lhs, const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * rhs);

/// Copy an array of srv/AddCalibrationPoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_thermal_calibration_interfaces
bool
thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence__copy(
  const thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * input,
  thermal_calibration_interfaces__srv__AddCalibrationPoint_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // THERMAL_CALIBRATION_INTERFACES__SRV__DETAIL__ADD_CALIBRATION_POINT__FUNCTIONS_H_
