// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from g1_interface:srv/G1Modes.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_MODES__STRUCT_H_
#define G1_INTERFACE__SRV__DETAIL__G1_MODES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request_data'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/G1Modes in the package g1_interface.
typedef struct g1_interface__srv__G1Modes_Request
{
  rosidl_runtime_c__String request_data;
} g1_interface__srv__G1Modes_Request;

// Struct for a sequence of g1_interface__srv__G1Modes_Request.
typedef struct g1_interface__srv__G1Modes_Request__Sequence
{
  g1_interface__srv__G1Modes_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} g1_interface__srv__G1Modes_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/G1Modes in the package g1_interface.
typedef struct g1_interface__srv__G1Modes_Response
{
  bool success;
  rosidl_runtime_c__String reason;
} g1_interface__srv__G1Modes_Response;

// Struct for a sequence of g1_interface__srv__G1Modes_Response.
typedef struct g1_interface__srv__G1Modes_Response__Sequence
{
  g1_interface__srv__G1Modes_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} g1_interface__srv__G1Modes_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // G1_INTERFACE__SRV__DETAIL__G1_MODES__STRUCT_H_
