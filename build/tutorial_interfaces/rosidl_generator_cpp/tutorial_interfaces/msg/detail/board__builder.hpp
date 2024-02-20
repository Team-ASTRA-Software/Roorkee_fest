// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/board__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Board_j
{
public:
  explicit Init_Board_j(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Board j(::tutorial_interfaces::msg::Board::_j_type arg)
  {
    msg_.j = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_z
{
public:
  explicit Init_Board_z(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_j z(::tutorial_interfaces::msg::Board::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Board_j(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_x
{
public:
  explicit Init_Board_x(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_z x(::tutorial_interfaces::msg::Board::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Board_z(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_c
{
public:
  explicit Init_Board_c(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_x c(::tutorial_interfaces::msg::Board::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_Board_x(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_e
{
public:
  explicit Init_Board_e(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_c e(::tutorial_interfaces::msg::Board::_e_type arg)
  {
    msg_.e = std::move(arg);
    return Init_Board_c(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_q
{
public:
  explicit Init_Board_q(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_e q(::tutorial_interfaces::msg::Board::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_Board_e(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_d
{
public:
  explicit Init_Board_d(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_q d(::tutorial_interfaces::msg::Board::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_Board_q(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_s
{
public:
  explicit Init_Board_s(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_d s(::tutorial_interfaces::msg::Board::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_Board_d(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_a
{
public:
  explicit Init_Board_a(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_s a(::tutorial_interfaces::msg::Board::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Board_s(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_w
{
public:
  explicit Init_Board_w(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_a w(::tutorial_interfaces::msg::Board::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Board_a(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_left_ctrl
{
public:
  explicit Init_Board_key_left_ctrl(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_w key_left_ctrl(::tutorial_interfaces::msg::Board::_key_left_ctrl_type arg)
  {
    msg_.key_left_ctrl = std::move(arg);
    return Init_Board_w(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_left_shift
{
public:
  explicit Init_Board_key_left_shift(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_key_left_ctrl key_left_shift(::tutorial_interfaces::msg::Board::_key_left_shift_type arg)
  {
    msg_.key_left_shift = std::move(arg);
    return Init_Board_key_left_ctrl(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_right
{
public:
  explicit Init_Board_key_right(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_key_left_shift key_right(::tutorial_interfaces::msg::Board::_key_right_type arg)
  {
    msg_.key_right = std::move(arg);
    return Init_Board_key_left_shift(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_left
{
public:
  explicit Init_Board_key_left(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_key_right key_left(::tutorial_interfaces::msg::Board::_key_left_type arg)
  {
    msg_.key_left = std::move(arg);
    return Init_Board_key_right(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_down
{
public:
  explicit Init_Board_key_down(::tutorial_interfaces::msg::Board & msg)
  : msg_(msg)
  {}
  Init_Board_key_left key_down(::tutorial_interfaces::msg::Board::_key_down_type arg)
  {
    msg_.key_down = std::move(arg);
    return Init_Board_key_left(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

class Init_Board_key_up
{
public:
  Init_Board_key_up()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Board_key_down key_up(::tutorial_interfaces::msg::Board::_key_up_type arg)
  {
    msg_.key_up = std::move(arg);
    return Init_Board_key_down(msg_);
  }

private:
  ::tutorial_interfaces::msg::Board msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Board>()
{
  return tutorial_interfaces::msg::builder::Init_Board_key_up();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__BUILDER_HPP_
