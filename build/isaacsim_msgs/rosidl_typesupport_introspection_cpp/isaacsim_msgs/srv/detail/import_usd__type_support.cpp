// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from isaacsim_msgs:srv/ImportUsd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "isaacsim_msgs/srv/detail/import_usd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace isaacsim_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ImportUsd_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) isaacsim_msgs::srv::ImportUsd_Request(_init);
}

void ImportUsd_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<isaacsim_msgs::srv::ImportUsd_Request *>(message_memory);
  typed_message->~ImportUsd_Request();
}

<<<<<<< HEAD
static const ::rosidl_typesupport_introspection_cpp::MessageMember ImportUsd_Request_message_member_array[4] = {
=======
size_t size_function__ImportUsd_Request__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__ImportUsd_Request__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__ImportUsd_Request__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ImportUsd_Request__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ImportUsd_Request__position(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ImportUsd_Request__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ImportUsd_Request__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ImportUsd_Request__orientation(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ImportUsd_Request__orientation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ImportUsd_Request__orientation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ImportUsd_Request__orientation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ImportUsd_Request__orientation(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ImportUsd_Request__orientation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ImportUsd_Request__orientation(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ImportUsd_Request_message_member_array[6] = {
>>>>>>> an
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "usd_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, usd_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "prim_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, prim_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "control",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, control),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
<<<<<<< HEAD
=======
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__ImportUsd_Request__position,  // size() function pointer
    get_const_function__ImportUsd_Request__position,  // get_const(index) function pointer
    get_function__ImportUsd_Request__position,  // get(index) function pointer
    fetch_function__ImportUsd_Request__position,  // fetch(index, &value) function pointer
    assign_function__ImportUsd_Request__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "orientation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Request, orientation),  // bytes offset in struct
    nullptr,  // default value
    size_function__ImportUsd_Request__orientation,  // size() function pointer
    get_const_function__ImportUsd_Request__orientation,  // get_const(index) function pointer
    get_function__ImportUsd_Request__orientation,  // get(index) function pointer
    fetch_function__ImportUsd_Request__orientation,  // fetch(index, &value) function pointer
    assign_function__ImportUsd_Request__orientation,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
>>>>>>> an
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ImportUsd_Request_message_members = {
  "isaacsim_msgs::srv",  // message namespace
  "ImportUsd_Request",  // message name
<<<<<<< HEAD
  4,  // number of fields
=======
  6,  // number of fields
>>>>>>> an
  sizeof(isaacsim_msgs::srv::ImportUsd_Request),
  ImportUsd_Request_message_member_array,  // message members
  ImportUsd_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ImportUsd_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ImportUsd_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ImportUsd_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace isaacsim_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<isaacsim_msgs::srv::ImportUsd_Request>()
{
  return &::isaacsim_msgs::srv::rosidl_typesupport_introspection_cpp::ImportUsd_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, isaacsim_msgs, srv, ImportUsd_Request)() {
  return &::isaacsim_msgs::srv::rosidl_typesupport_introspection_cpp::ImportUsd_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "isaacsim_msgs/srv/detail/import_usd__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace isaacsim_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ImportUsd_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) isaacsim_msgs::srv::ImportUsd_Response(_init);
}

void ImportUsd_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<isaacsim_msgs::srv::ImportUsd_Response *>(message_memory);
  typed_message->~ImportUsd_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ImportUsd_Response_message_member_array[1] = {
  {
    "ret",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(isaacsim_msgs::srv::ImportUsd_Response, ret),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ImportUsd_Response_message_members = {
  "isaacsim_msgs::srv",  // message namespace
  "ImportUsd_Response",  // message name
  1,  // number of fields
  sizeof(isaacsim_msgs::srv::ImportUsd_Response),
  ImportUsd_Response_message_member_array,  // message members
  ImportUsd_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ImportUsd_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ImportUsd_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ImportUsd_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace isaacsim_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<isaacsim_msgs::srv::ImportUsd_Response>()
{
  return &::isaacsim_msgs::srv::rosidl_typesupport_introspection_cpp::ImportUsd_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, isaacsim_msgs, srv, ImportUsd_Response)() {
  return &::isaacsim_msgs::srv::rosidl_typesupport_introspection_cpp::ImportUsd_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "isaacsim_msgs/srv/detail/import_usd__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace isaacsim_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers ImportUsd_service_members = {
  "isaacsim_msgs::srv",  // service namespace
  "ImportUsd",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<isaacsim_msgs::srv::ImportUsd>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t ImportUsd_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ImportUsd_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace isaacsim_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<isaacsim_msgs::srv::ImportUsd>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::isaacsim_msgs::srv::rosidl_typesupport_introspection_cpp::ImportUsd_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::isaacsim_msgs::srv::ImportUsd_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::isaacsim_msgs::srv::ImportUsd_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, isaacsim_msgs, srv, ImportUsd)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<isaacsim_msgs::srv::ImportUsd>();
}

#ifdef __cplusplus
}
#endif
