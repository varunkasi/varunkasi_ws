// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from thermal_calibration_interfaces:srv/ClearCalibrationData.idl
// generated code does not contain a copyright notice
#include "thermal_calibration_interfaces/srv/detail/clear_calibration_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__init(thermal_calibration_interfaces__srv__ClearCalibrationData_Request * msg)
{
  if (!msg) {
    return false;
  }
  // confirm
  return true;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__fini(thermal_calibration_interfaces__srv__ClearCalibrationData_Request * msg)
{
  if (!msg) {
    return;
  }
  // confirm
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__are_equal(const thermal_calibration_interfaces__srv__ClearCalibrationData_Request * lhs, const thermal_calibration_interfaces__srv__ClearCalibrationData_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // confirm
  if (lhs->confirm != rhs->confirm) {
    return false;
  }
  return true;
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__copy(
  const thermal_calibration_interfaces__srv__ClearCalibrationData_Request * input,
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // confirm
  output->confirm = input->confirm;
  return true;
}

thermal_calibration_interfaces__srv__ClearCalibrationData_Request *
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request * msg = (thermal_calibration_interfaces__srv__ClearCalibrationData_Request *)allocator.allocate(sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Request));
  bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__destroy(thermal_calibration_interfaces__srv__ClearCalibrationData_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thermal_calibration_interfaces__srv__ClearCalibrationData_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__init(thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request * data = NULL;

  if (size) {
    data = (thermal_calibration_interfaces__srv__ClearCalibrationData_Request *)allocator.zero_allocate(size, sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thermal_calibration_interfaces__srv__ClearCalibrationData_Request__fini(&data[i - 1]);
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
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__fini(thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * array)
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
      thermal_calibration_interfaces__srv__ClearCalibrationData_Request__fini(&array->data[i]);
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

thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence *
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * array = (thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence *)allocator.allocate(sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__destroy(thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__are_equal(const thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * lhs, const thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence__copy(
  const thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * input,
  thermal_calibration_interfaces__srv__ClearCalibrationData_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thermal_calibration_interfaces__srv__ClearCalibrationData_Request * data =
      (thermal_calibration_interfaces__srv__ClearCalibrationData_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thermal_calibration_interfaces__srv__ClearCalibrationData_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__init(thermal_calibration_interfaces__srv__ClearCalibrationData_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(msg);
    return false;
  }
  return true;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(thermal_calibration_interfaces__srv__ClearCalibrationData_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__are_equal(const thermal_calibration_interfaces__srv__ClearCalibrationData_Response * lhs, const thermal_calibration_interfaces__srv__ClearCalibrationData_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__copy(
  const thermal_calibration_interfaces__srv__ClearCalibrationData_Response * input,
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

thermal_calibration_interfaces__srv__ClearCalibrationData_Response *
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response * msg = (thermal_calibration_interfaces__srv__ClearCalibrationData_Response *)allocator.allocate(sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Response));
  bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__destroy(thermal_calibration_interfaces__srv__ClearCalibrationData_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__init(thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response * data = NULL;

  if (size) {
    data = (thermal_calibration_interfaces__srv__ClearCalibrationData_Response *)allocator.zero_allocate(size, sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(&data[i - 1]);
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
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__fini(thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * array)
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
      thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(&array->data[i]);
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

thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence *
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * array = (thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence *)allocator.allocate(sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__destroy(thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__are_equal(const thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * lhs, const thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence__copy(
  const thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * input,
  thermal_calibration_interfaces__srv__ClearCalibrationData_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(thermal_calibration_interfaces__srv__ClearCalibrationData_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    thermal_calibration_interfaces__srv__ClearCalibrationData_Response * data =
      (thermal_calibration_interfaces__srv__ClearCalibrationData_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          thermal_calibration_interfaces__srv__ClearCalibrationData_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!thermal_calibration_interfaces__srv__ClearCalibrationData_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
