// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from g1_interface:srv/G1Light.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_LIGHT__STRUCT_H_
#define G1_INTERFACE__SRV__DETAIL__G1_LIGHT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/G1Light in the package g1_interface.
typedef struct g1_interface__srv__G1Light_Request
{
  int32_t light_level;
} g1_interface__srv__G1Light_Request;

// Struct for a sequence of g1_interface__srv__G1Light_Request.
typedef struct g1_interface__srv__G1Light_Request__Sequence
{
  g1_interface__srv__G1Light_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} g1_interface__srv__G1Light_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/G1Light in the package g1_interface.
typedef struct g1_interface__srv__G1Light_Response
{
  bool success;
  rosidl_runtime_c__String reason;
} g1_interface__srv__G1Light_Response;

// Struct for a sequence of g1_interface__srv__G1Light_Response.
typedef struct g1_interface__srv__G1Light_Response__Sequence
{
  g1_interface__srv__G1Light_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} g1_interface__srv__G1Light_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // G1_INTERFACE__SRV__DETAIL__G1_LIGHT__STRUCT_H_
