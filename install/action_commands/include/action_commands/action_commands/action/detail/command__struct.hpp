// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from action_commands:action/Command.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_HPP_
#define ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_Goal __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_Goal __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Goal_
{
  using Type = Command_Goal_<ContainerAllocator>;

  explicit Command_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->param1 = 0.0f;
      this->param2 = 0.0f;
      this->param3 = 0.0f;
      this->param4 = 0.0f;
      this->param5 = 0.0;
      this->param6 = 0.0;
      this->param7 = 0.0f;
      this->command = 0ul;
      this->target_system = 0;
      this->target_component = 0;
      this->source_system = 0;
      this->source_component = 0;
      this->confirmation = 0;
      this->from_external = false;
    }
  }

  explicit Command_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->param1 = 0.0f;
      this->param2 = 0.0f;
      this->param3 = 0.0f;
      this->param4 = 0.0f;
      this->param5 = 0.0;
      this->param6 = 0.0;
      this->param7 = 0.0f;
      this->command = 0ul;
      this->target_system = 0;
      this->target_component = 0;
      this->source_system = 0;
      this->source_component = 0;
      this->confirmation = 0;
      this->from_external = false;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _param1_type =
    float;
  _param1_type param1;
  using _param2_type =
    float;
  _param2_type param2;
  using _param3_type =
    float;
  _param3_type param3;
  using _param4_type =
    float;
  _param4_type param4;
  using _param5_type =
    double;
  _param5_type param5;
  using _param6_type =
    double;
  _param6_type param6;
  using _param7_type =
    float;
  _param7_type param7;
  using _command_type =
    uint32_t;
  _command_type command;
  using _target_system_type =
    uint8_t;
  _target_system_type target_system;
  using _target_component_type =
    uint8_t;
  _target_component_type target_component;
  using _source_system_type =
    uint8_t;
  _source_system_type source_system;
  using _source_component_type =
    uint16_t;
  _source_component_type source_component;
  using _confirmation_type =
    uint8_t;
  _confirmation_type confirmation;
  using _from_external_type =
    bool;
  _from_external_type from_external;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__param1(
    const float & _arg)
  {
    this->param1 = _arg;
    return *this;
  }
  Type & set__param2(
    const float & _arg)
  {
    this->param2 = _arg;
    return *this;
  }
  Type & set__param3(
    const float & _arg)
  {
    this->param3 = _arg;
    return *this;
  }
  Type & set__param4(
    const float & _arg)
  {
    this->param4 = _arg;
    return *this;
  }
  Type & set__param5(
    const double & _arg)
  {
    this->param5 = _arg;
    return *this;
  }
  Type & set__param6(
    const double & _arg)
  {
    this->param6 = _arg;
    return *this;
  }
  Type & set__param7(
    const float & _arg)
  {
    this->param7 = _arg;
    return *this;
  }
  Type & set__command(
    const uint32_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__target_system(
    const uint8_t & _arg)
  {
    this->target_system = _arg;
    return *this;
  }
  Type & set__target_component(
    const uint8_t & _arg)
  {
    this->target_component = _arg;
    return *this;
  }
  Type & set__source_system(
    const uint8_t & _arg)
  {
    this->source_system = _arg;
    return *this;
  }
  Type & set__source_component(
    const uint16_t & _arg)
  {
    this->source_component = _arg;
    return *this;
  }
  Type & set__confirmation(
    const uint8_t & _arg)
  {
    this->confirmation = _arg;
    return *this;
  }
  Type & set__from_external(
    const bool & _arg)
  {
    this->from_external = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_Goal
    std::shared_ptr<action_commands::action::Command_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_Goal
    std::shared_ptr<action_commands::action::Command_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Goal_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->param1 != other.param1) {
      return false;
    }
    if (this->param2 != other.param2) {
      return false;
    }
    if (this->param3 != other.param3) {
      return false;
    }
    if (this->param4 != other.param4) {
      return false;
    }
    if (this->param5 != other.param5) {
      return false;
    }
    if (this->param6 != other.param6) {
      return false;
    }
    if (this->param7 != other.param7) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->target_system != other.target_system) {
      return false;
    }
    if (this->target_component != other.target_component) {
      return false;
    }
    if (this->source_system != other.source_system) {
      return false;
    }
    if (this->source_component != other.source_component) {
      return false;
    }
    if (this->confirmation != other.confirmation) {
      return false;
    }
    if (this->from_external != other.from_external) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Goal_

// alias to use template instance with default allocator
using Command_Goal =
  action_commands::action::Command_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_Result __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_Result __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Result_
{
  using Type = Command_Result_<ContainerAllocator>;

  explicit Command_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming_state = 0;
      this->nav_state = 0;
    }
  }

  explicit Command_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming_state = 0;
      this->nav_state = 0;
    }
  }

  // field types and members
  using _arming_state_type =
    uint8_t;
  _arming_state_type arming_state;
  using _nav_state_type =
    uint8_t;
  _nav_state_type nav_state;

  // setters for named parameter idiom
  Type & set__arming_state(
    const uint8_t & _arg)
  {
    this->arming_state = _arg;
    return *this;
  }
  Type & set__nav_state(
    const uint8_t & _arg)
  {
    this->nav_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_Result
    std::shared_ptr<action_commands::action::Command_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_Result
    std::shared_ptr<action_commands::action::Command_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Result_ & other) const
  {
    if (this->arming_state != other.arming_state) {
      return false;
    }
    if (this->nav_state != other.nav_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Result_

// alias to use template instance with default allocator
using Command_Result =
  action_commands::action::Command_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_Feedback __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_Feedback_
{
  using Type = Command_Feedback_<ContainerAllocator>;

  explicit Command_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming_state_present = 0;
      this->nav_state_present = 0;
    }
  }

  explicit Command_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arming_state_present = 0;
      this->nav_state_present = 0;
    }
  }

  // field types and members
  using _arming_state_present_type =
    uint8_t;
  _arming_state_present_type arming_state_present;
  using _nav_state_present_type =
    uint8_t;
  _nav_state_present_type nav_state_present;

  // setters for named parameter idiom
  Type & set__arming_state_present(
    const uint8_t & _arg)
  {
    this->arming_state_present = _arg;
    return *this;
  }
  Type & set__nav_state_present(
    const uint8_t & _arg)
  {
    this->nav_state_present = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_Feedback
    std::shared_ptr<action_commands::action::Command_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_Feedback
    std::shared_ptr<action_commands::action::Command_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_Feedback_ & other) const
  {
    if (this->arming_state_present != other.arming_state_present) {
      return false;
    }
    if (this->nav_state_present != other.nav_state_present) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_Feedback_

// alias to use template instance with default allocator
using Command_Feedback =
  action_commands::action::Command_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "action_commands/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_SendGoal_Request __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_SendGoal_Request_
{
  using Type = Command_SendGoal_Request_<ContainerAllocator>;

  explicit Command_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Command_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    action_commands::action::Command_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const action_commands::action::Command_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_SendGoal_Request
    std::shared_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_SendGoal_Request
    std::shared_ptr<action_commands::action::Command_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_SendGoal_Request_

// alias to use template instance with default allocator
using Command_SendGoal_Request =
  action_commands::action::Command_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_SendGoal_Response __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_SendGoal_Response_
{
  using Type = Command_SendGoal_Response_<ContainerAllocator>;

  explicit Command_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Command_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_SendGoal_Response
    std::shared_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_SendGoal_Response
    std::shared_ptr<action_commands::action::Command_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_SendGoal_Response_

// alias to use template instance with default allocator
using Command_SendGoal_Response =
  action_commands::action::Command_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands

namespace action_commands
{

namespace action
{

struct Command_SendGoal
{
  using Request = action_commands::action::Command_SendGoal_Request;
  using Response = action_commands::action::Command_SendGoal_Response;
};

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_GetResult_Request __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_GetResult_Request_
{
  using Type = Command_GetResult_Request_<ContainerAllocator>;

  explicit Command_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Command_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_GetResult_Request
    std::shared_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_GetResult_Request
    std::shared_ptr<action_commands::action::Command_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_GetResult_Request_

// alias to use template instance with default allocator
using Command_GetResult_Request =
  action_commands::action::Command_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_GetResult_Response __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_GetResult_Response_
{
  using Type = Command_GetResult_Response_<ContainerAllocator>;

  explicit Command_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Command_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    action_commands::action::Command_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const action_commands::action::Command_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_GetResult_Response
    std::shared_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_GetResult_Response
    std::shared_ptr<action_commands::action::Command_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_GetResult_Response_

// alias to use template instance with default allocator
using Command_GetResult_Response =
  action_commands::action::Command_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands

namespace action_commands
{

namespace action
{

struct Command_GetResult
{
  using Request = action_commands::action::Command_GetResult_Request;
  using Response = action_commands::action::Command_GetResult_Response;
};

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "action_commands/action/detail/command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__Command_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__Command_FeedbackMessage __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Command_FeedbackMessage_
{
  using Type = Command_FeedbackMessage_<ContainerAllocator>;

  explicit Command_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Command_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    action_commands::action::Command_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const action_commands::action::Command_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::Command_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::Command_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::Command_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__Command_FeedbackMessage
    std::shared_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__Command_FeedbackMessage
    std::shared_ptr<action_commands::action::Command_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_FeedbackMessage_

// alias to use template instance with default allocator
using Command_FeedbackMessage =
  action_commands::action::Command_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace action_commands
{

namespace action
{

struct Command
{
  /// The goal message defined in the action definition.
  using Goal = action_commands::action::Command_Goal;
  /// The result message defined in the action definition.
  using Result = action_commands::action::Command_Result;
  /// The feedback message defined in the action definition.
  using Feedback = action_commands::action::Command_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = action_commands::action::Command_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = action_commands::action::Command_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = action_commands::action::Command_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Command Command;

}  // namespace action

}  // namespace action_commands

#endif  // ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_HPP_
