// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_commands:action/FollowGPSWaypoints.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__BUILDER_HPP_
#define ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_commands/action/detail/follow_gps_waypoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_Goal_gps_poses
{
public:
  explicit Init_FollowGPSWaypoints_Goal_gps_poses(::action_commands::action::FollowGPSWaypoints_Goal & msg)
  : msg_(msg)
  {}
  ::action_commands::action::FollowGPSWaypoints_Goal gps_poses(::action_commands::action::FollowGPSWaypoints_Goal::_gps_poses_type arg)
  {
    msg_.gps_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_Goal msg_;
};

class Init_FollowGPSWaypoints_Goal_goal_index
{
public:
  explicit Init_FollowGPSWaypoints_Goal_goal_index(::action_commands::action::FollowGPSWaypoints_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowGPSWaypoints_Goal_gps_poses goal_index(::action_commands::action::FollowGPSWaypoints_Goal::_goal_index_type arg)
  {
    msg_.goal_index = std::move(arg);
    return Init_FollowGPSWaypoints_Goal_gps_poses(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_Goal msg_;
};

class Init_FollowGPSWaypoints_Goal_number_of_loops
{
public:
  Init_FollowGPSWaypoints_Goal_number_of_loops()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowGPSWaypoints_Goal_goal_index number_of_loops(::action_commands::action::FollowGPSWaypoints_Goal::_number_of_loops_type arg)
  {
    msg_.number_of_loops = std::move(arg);
    return Init_FollowGPSWaypoints_Goal_goal_index(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_Goal>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_Goal_number_of_loops();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_Result_missed_waypoints
{
public:
  Init_FollowGPSWaypoints_Result_missed_waypoints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_commands::action::FollowGPSWaypoints_Result missed_waypoints(::action_commands::action::FollowGPSWaypoints_Result::_missed_waypoints_type arg)
  {
    msg_.missed_waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_Result>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_Result_missed_waypoints();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_Feedback_current_waypoint
{
public:
  Init_FollowGPSWaypoints_Feedback_current_waypoint()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_commands::action::FollowGPSWaypoints_Feedback current_waypoint(::action_commands::action::FollowGPSWaypoints_Feedback::_current_waypoint_type arg)
  {
    msg_.current_waypoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_Feedback>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_Feedback_current_waypoint();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_SendGoal_Request_goal
{
public:
  explicit Init_FollowGPSWaypoints_SendGoal_Request_goal(::action_commands::action::FollowGPSWaypoints_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Request goal(::action_commands::action::FollowGPSWaypoints_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Request msg_;
};

class Init_FollowGPSWaypoints_SendGoal_Request_goal_id
{
public:
  Init_FollowGPSWaypoints_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowGPSWaypoints_SendGoal_Request_goal goal_id(::action_commands::action::FollowGPSWaypoints_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowGPSWaypoints_SendGoal_Request_goal(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_SendGoal_Request>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_SendGoal_Request_goal_id();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_SendGoal_Response_stamp
{
public:
  explicit Init_FollowGPSWaypoints_SendGoal_Response_stamp(::action_commands::action::FollowGPSWaypoints_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Response stamp(::action_commands::action::FollowGPSWaypoints_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Response msg_;
};

class Init_FollowGPSWaypoints_SendGoal_Response_accepted
{
public:
  Init_FollowGPSWaypoints_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowGPSWaypoints_SendGoal_Response_stamp accepted(::action_commands::action::FollowGPSWaypoints_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FollowGPSWaypoints_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_SendGoal_Response>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_SendGoal_Response_accepted();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_GetResult_Request_goal_id
{
public:
  Init_FollowGPSWaypoints_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_commands::action::FollowGPSWaypoints_GetResult_Request goal_id(::action_commands::action::FollowGPSWaypoints_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_GetResult_Request>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_GetResult_Request_goal_id();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_GetResult_Response_result
{
public:
  explicit Init_FollowGPSWaypoints_GetResult_Response_result(::action_commands::action::FollowGPSWaypoints_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_commands::action::FollowGPSWaypoints_GetResult_Response result(::action_commands::action::FollowGPSWaypoints_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_GetResult_Response msg_;
};

class Init_FollowGPSWaypoints_GetResult_Response_status
{
public:
  Init_FollowGPSWaypoints_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowGPSWaypoints_GetResult_Response_result status(::action_commands::action::FollowGPSWaypoints_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowGPSWaypoints_GetResult_Response_result(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_GetResult_Response>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_GetResult_Response_status();
}

}  // namespace action_commands


namespace action_commands
{

namespace action
{

namespace builder
{

class Init_FollowGPSWaypoints_FeedbackMessage_feedback
{
public:
  explicit Init_FollowGPSWaypoints_FeedbackMessage_feedback(::action_commands::action::FollowGPSWaypoints_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_commands::action::FollowGPSWaypoints_FeedbackMessage feedback(::action_commands::action::FollowGPSWaypoints_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_FeedbackMessage msg_;
};

class Init_FollowGPSWaypoints_FeedbackMessage_goal_id
{
public:
  Init_FollowGPSWaypoints_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowGPSWaypoints_FeedbackMessage_feedback goal_id(::action_commands::action::FollowGPSWaypoints_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowGPSWaypoints_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_commands::action::FollowGPSWaypoints_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_commands::action::FollowGPSWaypoints_FeedbackMessage>()
{
  return action_commands::action::builder::Init_FollowGPSWaypoints_FeedbackMessage_goal_id();
}

}  // namespace action_commands

#endif  // ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__BUILDER_HPP_
