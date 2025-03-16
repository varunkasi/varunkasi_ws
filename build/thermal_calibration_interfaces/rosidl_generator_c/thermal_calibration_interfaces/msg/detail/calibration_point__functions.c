// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
// generated code does not contain a copyright notice
#include "thermal_calibration_interfaces/msg/detail/calibration_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "rosidl_runtime_c/string_functions.h"

bool
thermal_calibration_interfaces__msg__CalibrationPoint__init(thermal_calibration_interfaces__msg__CalibrationPoint * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // x
  // y
  // raw_value
  // reference_temp
  // timestamp
  if (!rosidl_runtime_c__String__init(&msg->timestamp)) {
    thermal_calibration_interfaces__msg__CalibrationPoint__fini(msg);
    return false;
  }
  return true;
}

void
thermal_calibration_interfaces__msg__CalibrationPoint__fini(thermal_calibration_interfaces__msg__CalibrationPoint * msg)
{
  if (!msg) {
    return;
  }
  // id
  // x
  // y
  // raw_value
  // reference_temp
  // timestamp
  rosidl_runtime_c__String__fini(&msg->timestamp);
}

bool
thermal_calibration_interfaces__msg__CalibrationPoint__are_equal(const thermal_calibration_interfaces__msg__CalibrationPoint * lhs, const thermal_calibration_interfaces__msg__CalibrationPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // raw_value
  if (lhs->raw_value != rhs->raw_value) {
    return false;
  }
  // reference_temp
  if (lhs->reference_temp != rhs->reference_temp) {
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
thermal_calibration_interfaces__msg__CalibrationPoint__copy(
  const thermal_calibration_interfaces__msg__CalibrationPoint * input,
  thermal_calibration_interfaces__msg__CalibrationPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // raw_value
  output->raw_value = input->raw_value;
  // reference_temp
  output->reference_temp = input->reference_temp;
  // timestamp
  if (!rosidl_runtime_c__String__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

thermal_calibration_interfaces__msg__CalibrationPoint *
thermal_calibration_interfaces__msg__CalibrationPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationPoint * msg = (thermal_calibration_interfaces__msg__CalibrationPoint *)allocator.allocate(sizeof(thermal_calibration_interfaces__msg__CalibrationPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thermal_calibration_interfaces__msg__CalibrationPoint));
  bool success = thermal_calibration_interfaces__msg__CalibrationPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thermal_calibration_interfaces__msg__CalibrationPoint__destroy(thermal_calibration_interfaces__msg__CalibrationPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thermal_calibration_interfaces__msg__CalibrationPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__init(thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationPoint * data = NULL;

  if (size) {
    data = (thermal_calibration_interfaces__msg__CalibrationPoint *)allocator.zero_allocate(size, sizeof(thermal_calibration_interfaces__msg__CalibrationPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thermal_calibration_interfaces__msg__CalibrationPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thermal_calibration_interfaces__msg__CalibrationPoint__fini(&data[i - 1]);
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
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__fini(thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * array)
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
      thermal_calibration_interfaces__msg__CalibrationPoint__fini(&array->data[i]);
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

thermal_calibration_interfaces__msg__CalibrationPoint__Sequence *
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * array = (thermal_calibration_interfaces__msg__CalibrationPoint__Sequence *)allocator.allocate(sizeof(thermal_calibration_interfaces__msg__CalibrationPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__destroy(thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__are_equal(const thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * lhs, const thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thermal_calibration_interfaces__msg__CalibrationPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thermal_calibration_interfaces__msg__CalibrationPoint__Sequence__copy(
  const thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * input,
  thermal_calibration_interfaces__msg__CalibrationPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thermal_calibration_interfaces__msg__CalibrationPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thermal_calibration_interfaces__msg__CalibrationPoint * data =
      (thermal_calibration_interfaces__msg__CalibrationPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thermal_calibration_interfaces__msg__CalibrationPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thermal_calibration_interfaces__msg__CalibrationPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thermal_calibration_interfaces__msg__CalibrationPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
