// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from action_commands:action/FollowGPSWaypoints.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_HPP_
#define ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'gps_poses'
#include "geometry_msgs/msg/detail/geo_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Goal __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Goal __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_Goal_
{
  using Type = FollowGPSWaypoints_Goal_<ContainerAllocator>;

  explicit FollowGPSWaypoints_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->goal_index = 0ul;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->number_of_loops = 0ul;
      this->goal_index = 0ul;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number_of_loops = 0ul;
    }
  }

  explicit FollowGPSWaypoints_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->goal_index = 0ul;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->number_of_loops = 0ul;
      this->goal_index = 0ul;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number_of_loops = 0ul;
    }
  }

  // field types and members
  using _number_of_loops_type =
    uint32_t;
  _number_of_loops_type number_of_loops;
  using _goal_index_type =
    uint32_t;
  _goal_index_type goal_index;
  using _gps_poses_type =
    std::vector<geometry_msgs::msg::GeoPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::GeoPose_<ContainerAllocator>>>;
  _gps_poses_type gps_poses;

  // setters for named parameter idiom
  Type & set__number_of_loops(
    const uint32_t & _arg)
  {
    this->number_of_loops = _arg;
    return *this;
  }
  Type & set__goal_index(
    const uint32_t & _arg)
  {
    this->goal_index = _arg;
    return *this;
  }
  Type & set__gps_poses(
    const std::vector<geometry_msgs::msg::GeoPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::GeoPose_<ContainerAllocator>>> & _arg)
  {
    this->gps_poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Goal
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Goal
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_Goal_ & other) const
  {
    if (this->number_of_loops != other.number_of_loops) {
      return false;
    }
    if (this->goal_index != other.goal_index) {
      return false;
    }
    if (this->gps_poses != other.gps_poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_Goal_

// alias to use template instance with default allocator
using FollowGPSWaypoints_Goal =
  action_commands::action::FollowGPSWaypoints_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'missed_waypoints'
#include "action_commands/msg/detail/missed_waypoint__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Result __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Result __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_Result_
{
  using Type = FollowGPSWaypoints_Result_<ContainerAllocator>;

  explicit FollowGPSWaypoints_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FollowGPSWaypoints_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _missed_waypoints_type =
    std::vector<action_commands::msg::MissedWaypoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<action_commands::msg::MissedWaypoint_<ContainerAllocator>>>;
  _missed_waypoints_type missed_waypoints;

  // setters for named parameter idiom
  Type & set__missed_waypoints(
    const std::vector<action_commands::msg::MissedWaypoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<action_commands::msg::MissedWaypoint_<ContainerAllocator>>> & _arg)
  {
    this->missed_waypoints = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t NONE =
    0u;
  static constexpr uint16_t UNKNOWN =
    600u;
  static constexpr uint16_t TASK_EXECUTOR_FAILED =
    601u;

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Result
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Result
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_Result_ & other) const
  {
    if (this->missed_waypoints != other.missed_waypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_Result_

// alias to use template instance with default allocator
using FollowGPSWaypoints_Result =
  action_commands::action::FollowGPSWaypoints_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t FollowGPSWaypoints_Result_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t FollowGPSWaypoints_Result_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t FollowGPSWaypoints_Result_<ContainerAllocator>::TASK_EXECUTOR_FAILED;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace action_commands


#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_Feedback __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_Feedback_
{
  using Type = FollowGPSWaypoints_Feedback_<ContainerAllocator>;

  explicit FollowGPSWaypoints_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_waypoint = 0ul;
    }
  }

  explicit FollowGPSWaypoints_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_waypoint = 0ul;
    }
  }

  // field types and members
  using _current_waypoint_type =
    uint32_t;
  _current_waypoint_type current_waypoint;

  // setters for named parameter idiom
  Type & set__current_waypoint(
    const uint32_t & _arg)
  {
    this->current_waypoint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Feedback
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_Feedback
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_Feedback_ & other) const
  {
    if (this->current_waypoint != other.current_waypoint) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_Feedback_

// alias to use template instance with default allocator
using FollowGPSWaypoints_Feedback =
  action_commands::action::FollowGPSWaypoints_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "action_commands/action/detail/follow_gps_waypoints__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Request __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_SendGoal_Request_
{
  using Type = FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>;

  explicit FollowGPSWaypoints_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit FollowGPSWaypoints_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const action_commands::action::FollowGPSWaypoints_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Request
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Request
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_SendGoal_Request_

// alias to use template instance with default allocator
using FollowGPSWaypoints_SendGoal_Request =
  action_commands::action::FollowGPSWaypoints_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Response __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_SendGoal_Response_
{
  using Type = FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>;

  explicit FollowGPSWaypoints_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit FollowGPSWaypoints_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Response
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_SendGoal_Response
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_SendGoal_Response_

// alias to use template instance with default allocator
using FollowGPSWaypoints_SendGoal_Response =
  action_commands::action::FollowGPSWaypoints_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands

namespace action_commands
{

namespace action
{

struct FollowGPSWaypoints_SendGoal
{
  using Request = action_commands::action::FollowGPSWaypoints_SendGoal_Request;
  using Response = action_commands::action::FollowGPSWaypoints_SendGoal_Response;
};

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Request __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_GetResult_Request_
{
  using Type = FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>;

  explicit FollowGPSWaypoints_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit FollowGPSWaypoints_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Request
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Request
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_GetResult_Request_

// alias to use template instance with default allocator
using FollowGPSWaypoints_GetResult_Request =
  action_commands::action::FollowGPSWaypoints_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/follow_gps_waypoints__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Response __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_GetResult_Response_
{
  using Type = FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>;

  explicit FollowGPSWaypoints_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit FollowGPSWaypoints_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const action_commands::action::FollowGPSWaypoints_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Response
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_GetResult_Response
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_GetResult_Response_

// alias to use template instance with default allocator
using FollowGPSWaypoints_GetResult_Response =
  action_commands::action::FollowGPSWaypoints_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace action_commands

namespace action_commands
{

namespace action
{

struct FollowGPSWaypoints_GetResult
{
  using Request = action_commands::action::FollowGPSWaypoints_GetResult_Request;
  using Response = action_commands::action::FollowGPSWaypoints_GetResult_Response;
};

}  // namespace action

}  // namespace action_commands


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "action_commands/action/detail/follow_gps_waypoints__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__action_commands__action__FollowGPSWaypoints_FeedbackMessage __declspec(deprecated)
#endif

namespace action_commands
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct FollowGPSWaypoints_FeedbackMessage_
{
  using Type = FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>;

  explicit FollowGPSWaypoints_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit FollowGPSWaypoints_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const action_commands::action::FollowGPSWaypoints_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_FeedbackMessage
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_commands__action__FollowGPSWaypoints_FeedbackMessage
    std::shared_ptr<action_commands::action::FollowGPSWaypoints_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FollowGPSWaypoints_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const FollowGPSWaypoints_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FollowGPSWaypoints_FeedbackMessage_

// alias to use template instance with default allocator
using FollowGPSWaypoints_FeedbackMessage =
  action_commands::action::FollowGPSWaypoints_FeedbackMessage_<std::allocator<void>>;

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

struct FollowGPSWaypoints
{
  /// The goal message defined in the action definition.
  using Goal = action_commands::action::FollowGPSWaypoints_Goal;
  /// The result message defined in the action definition.
  using Result = action_commands::action::FollowGPSWaypoints_Result;
  /// The feedback message defined in the action definition.
  using Feedback = action_commands::action::FollowGPSWaypoints_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = action_commands::action::FollowGPSWaypoints_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = action_commands::action::FollowGPSWaypoints_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = action_commands::action::FollowGPSWaypoints_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct FollowGPSWaypoints FollowGPSWaypoints;

}  // namespace action

}  // namespace action_commands

#endif  // ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_HPP_
