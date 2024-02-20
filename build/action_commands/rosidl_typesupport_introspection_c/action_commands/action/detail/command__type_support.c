// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from action_commands:action/Command.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
#include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "action_commands/action/detail/command__functions.h"
#include "action_commands/action/detail/command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_Goal__init(message_memory);
}

void action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_fini_function(void * message_memory)
{
  action_commands__action__Command_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_member_array[15] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param6",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param6),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param7",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, param7),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_system",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, target_system),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_component",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, target_component),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_system",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, source_system),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_component",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, source_component),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confirmation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, confirmation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "from_external",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Goal, from_external),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_members = {
  "action_commands__action",  // message namespace
  "Command_Goal",  // message name
  15,  // number of fields
  sizeof(action_commands__action__Command_Goal),
  action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_member_array,  // message members
  action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_type_support_handle = {
  0,
  &action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Goal)() {
  if (!action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_Goal__rosidl_typesupport_introspection_c__Command_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_Result__init(message_memory);
}

void action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_fini_function(void * message_memory)
{
  action_commands__action__Command_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_member_array[2] = {
  {
    "arming_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Result, arming_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nav_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Result, nav_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_members = {
  "action_commands__action",  // message namespace
  "Command_Result",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_Result),
  action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_member_array,  // message members
  action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_type_support_handle = {
  0,
  &action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Result)() {
  if (!action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_Result__rosidl_typesupport_introspection_c__Command_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_Feedback__init(message_memory);
}

void action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_fini_function(void * message_memory)
{
  action_commands__action__Command_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_member_array[2] = {
  {
    "arming_state_present",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Feedback, arming_state_present),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nav_state_present",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_Feedback, nav_state_present),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_members = {
  "action_commands__action",  // message namespace
  "Command_Feedback",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_Feedback),
  action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_member_array,  // message members
  action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_type_support_handle = {
  0,
  &action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Feedback)() {
  if (!action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_Feedback__rosidl_typesupport_introspection_c__Command_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "action_commands/action/command.h"
// Member `goal`
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_SendGoal_Request__init(message_memory);
}

void action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_fini_function(void * message_memory)
{
  action_commands__action__Command_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_members = {
  "action_commands__action",  // message namespace
  "Command_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_SendGoal_Request),
  action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_member_array,  // message members
  action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_type_support_handle = {
  0,
  &action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Request)() {
  action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Goal)();
  if (!action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_SendGoal_Request__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_SendGoal_Response__init(message_memory);
}

void action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_fini_function(void * message_memory)
{
  action_commands__action__Command_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_members = {
  "action_commands__action",  // message namespace
  "Command_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_SendGoal_Response),
  action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_member_array,  // message members
  action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_type_support_handle = {
  0,
  &action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Response)() {
  action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_SendGoal_Response__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_members = {
  "action_commands__action",  // service namespace
  "Command_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_type_support_handle = {
  0,
  &action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal)() {
  if (!action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_type_support_handle.typesupport_identifier) {
    action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_SendGoal_Response)()->data;
  }

  return &action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_GetResult_Request__init(message_memory);
}

void action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_fini_function(void * message_memory)
{
  action_commands__action__Command_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_members = {
  "action_commands__action",  // message namespace
  "Command_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(action_commands__action__Command_GetResult_Request),
  action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_member_array,  // message members
  action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_type_support_handle = {
  0,
  &action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Request)() {
  action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_GetResult_Request__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "action_commands/action/command.h"
// Member `result`
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_GetResult_Response__init(message_memory);
}

void action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_fini_function(void * message_memory)
{
  action_commands__action__Command_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_members = {
  "action_commands__action",  // message namespace
  "Command_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_GetResult_Response),
  action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_member_array,  // message members
  action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_type_support_handle = {
  0,
  &action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Response)() {
  action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Result)();
  if (!action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_GetResult_Response__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_members = {
  "action_commands__action",  // service namespace
  "Command_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_type_support_handle = {
  0,
  &action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult)() {
  if (!action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_type_support_handle.typesupport_identifier) {
    action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_GetResult_Response)()->data;
  }

  return &action_commands__action__detail__command__rosidl_typesupport_introspection_c__Command_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"
// already included above
// #include "action_commands/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "action_commands/action/detail/command__functions.h"
// already included above
// #include "action_commands/action/detail/command__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "action_commands/action/command.h"
// Member `feedback`
// already included above
// #include "action_commands/action/detail/command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  action_commands__action__Command_FeedbackMessage__init(message_memory);
}

void action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_fini_function(void * message_memory)
{
  action_commands__action__Command_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(action_commands__action__Command_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_members = {
  "action_commands__action",  // message namespace
  "Command_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(action_commands__action__Command_FeedbackMessage),
  action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_member_array,  // message members
  action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_type_support_handle = {
  0,
  &action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_action_commands
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_FeedbackMessage)() {
  action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_commands, action, Command_Feedback)();
  if (!action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &action_commands__action__Command_FeedbackMessage__rosidl_typesupport_introspection_c__Command_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
