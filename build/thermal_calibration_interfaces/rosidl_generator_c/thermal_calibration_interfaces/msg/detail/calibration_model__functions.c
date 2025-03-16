// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
// generated code does not contain a copyright notice
#include "thermal_calibration_interfaces/msg/detail/calibration_model__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `model_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `parameters`
// Member `raw_value_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
thermal_calibration_interfaces__msg__CalibrationModel__init(thermal_calibration_interfaces__msg__CalibrationModel * msg)
{
  if (!msg) {
    return false;
  }
  // model_type
  if (!rosidl_runtime_c__String__init(&msg->model_type)) {
    thermal_calibration_interfaces__msg__CalibrationModel__fini(msg);
    return false;
  }
  // degree
  // parameters
  if (!rosidl_runtime_c__float__Sequence__init(&msg->parameters, 0)) {
    thermal_calibration_interfaces__msg__CalibrationModel__fini(msg);
    return false;
  }
  // r_squared
  // rmse
  // points_count
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    thermal_calibration_interfaces__msg__CalibrationModel__fini(msg);
    return false;
  }
  // raw_value_range
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->raw_value_range, 0)) {
    thermal_calibration_interfaces__msg__CalibrationModel__fini(msg);
    return false;
  }
  return true;
}

void
thermal_calibration_interfaces__msg__CalibrationModel__fini(thermal_calibration_interfaces__msg__CalibrationModel * msg)
{
  if (!msg) {
    return;
  }
  // model_type
  rosidl_runtime_c__String__fini(&msg->model_type);
  // degree
  // parameters
  rosidl_runtime_c__float__Sequence__fini(&msg->parameters);
  // r_squared
  // rmse
  // points_count
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // raw_value_range
  rosidl_runtime_c__uint16__Sequence__fini(&msg->raw_value_range);
}

bool
thermal_calibration_interfaces__msg__CalibrationModel__are_equal(const thermal_calibration_interfaces__msg__CalibrationModel * lhs, const thermal_calibration_interfaces__msg__CalibrationModel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // model_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->model_type), &(rhs->model_type)))
  {
    return false;
  }
  // degree
  if (lhs->degree != rhs->degree) {
    return false;
  }
  // parameters
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->parameters), &(rhs->parameters)))
  {
    return false;
  }
  // r_squared
  if (lhs->r_squared != rhs->r_squared) {
    return false;
  }
  // rmse
  if (lhs->rmse != rhs->rmse) {
    return false;
  }
  // points_count
  if (lhs->points_count != rhs->points_count) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // raw_value_range
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->raw_value_range), &(rhs->raw_value_range)))
  {
    return false;
  }
  return true;
}

bool
thermal_calibration_interfaces__msg__CalibrationModel__copy(
  const thermal_calibration_interfaces__msg__CalibrationModel * input,
  thermal_calibration_interfaces__msg__CalibrationModel * output)
{
  if (!input || !output) {
    return false;
  }
  // model_type
  if (!rosidl_runtime_c__String__copy(
      &(input->model_type), &(output->model_type)))
  {
    return false;
  }
  // degree
  output->degree = input->degree;
  // parameters
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->parameters), &(output->parameters)))
  {
    return false;
  }
  // r_squared
  output->r_squared = input->r_squared;
  // rmse
  output->rmse = input->rmse;
  // points_count
  output->points_count = input->points_count;
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // raw_value_range
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->raw_value_range), &(output->raw_value_range)))
  {
    return false;
  }
  return true;
}

thermal_calibration_interfaces__msg__CalibrationModel *
thermal_calibration_interfaces__msg__CalibrationModel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationModel * msg = (thermal_calibration_interfaces__msg__CalibrationModel *)allocator.allocate(sizeof(thermal_calibration_interfaces__msg__CalibrationModel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thermal_calibration_interfaces__msg__CalibrationModel));
  bool success = thermal_calibration_interfaces__msg__CalibrationModel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thermal_calibration_interfaces__msg__CalibrationModel__destroy(thermal_calibration_interfaces__msg__CalibrationModel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thermal_calibration_interfaces__msg__CalibrationModel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__init(thermal_calibration_interfaces__msg__CalibrationModel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationModel * data = NULL;

  if (size) {
    data = (thermal_calibration_interfaces__msg__CalibrationModel *)allocator.zero_allocate(size, sizeof(thermal_calibration_interfaces__msg__CalibrationModel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thermal_calibration_interfaces__msg__CalibrationModel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thermal_calibration_interfaces__msg__CalibrationModel__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__fini(thermal_calibration_interfaces__msg__CalibrationModel__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      thermal_calibration_interfaces__msg__CalibrationModel__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

thermal_calibration_interfaces__msg__CalibrationModel__Sequence *
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationModel__Sequence * array = (thermal_calibration_interfaces__msg__CalibrationModel__Sequence *)allocator.allocate(sizeof(thermal_calibration_interfaces__msg__CalibrationModel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thermal_calibration_interfaces__msg__CalibrationModel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__destroy(thermal_calibration_interfaces__msg__CalibrationModel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thermal_calibration_interfaces__msg__CalibrationModel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__are_equal(const thermal_calibration_interfaces__msg__CalibrationModel__Sequence * lhs, const thermal_calibration_interfaces__msg__CalibrationModel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thermal_calibration_interfaces__msg__CalibrationModel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thermal_calibration_interfaces__msg__CalibrationModel__Sequence__copy(
  const thermal_calibration_interfaces__msg__CalibrationModel__Sequence * input,
  thermal_calibration_interfaces__msg__CalibrationModel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thermal_calibration_interfaces__msg__CalibrationModel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thermal_calibration_interfaces__msg__CalibrationModel * data =
      (thermal_calibration_interfaces__msg__CalibrationModel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thermal_calibration_interfaces__msg__CalibrationModel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thermal_calibration_interfaces__msg__CalibrationModel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thermal_calibration_interfaces__msg__CalibrationModel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
