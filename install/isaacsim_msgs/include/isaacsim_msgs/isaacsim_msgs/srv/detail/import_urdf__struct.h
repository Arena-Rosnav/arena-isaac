// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from isaacsim_msgs:srv/ImportUrdf.idl
// generated code does not contain a copyright notice

#ifndef ISAACSIM_MSGS__SRV__DETAIL__IMPORT_URDF__STRUCT_H_
#define ISAACSIM_MSGS__SRV__DETAIL__IMPORT_URDF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'urdf_path'
// Member 'prim_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ImportUrdf in the package isaacsim_msgs.
typedef struct isaacsim_msgs__srv__ImportUrdf_Request
{
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String urdf_path;
  rosidl_runtime_c__String prim_path;
  bool control;
} isaacsim_msgs__srv__ImportUrdf_Request;

// Struct for a sequence of isaacsim_msgs__srv__ImportUrdf_Request.
typedef struct isaacsim_msgs__srv__ImportUrdf_Request__Sequence
{
  isaacsim_msgs__srv__ImportUrdf_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaacsim_msgs__srv__ImportUrdf_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'usd_path'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ImportUrdf in the package isaacsim_msgs.
typedef struct isaacsim_msgs__srv__ImportUrdf_Response
{
  rosidl_runtime_c__String usd_path;
} isaacsim_msgs__srv__ImportUrdf_Response;

// Struct for a sequence of isaacsim_msgs__srv__ImportUrdf_Response.
typedef struct isaacsim_msgs__srv__ImportUrdf_Response__Sequence
{
  isaacsim_msgs__srv__ImportUrdf_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaacsim_msgs__srv__ImportUrdf_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ISAACSIM_MSGS__SRV__DETAIL__IMPORT_URDF__STRUCT_H_