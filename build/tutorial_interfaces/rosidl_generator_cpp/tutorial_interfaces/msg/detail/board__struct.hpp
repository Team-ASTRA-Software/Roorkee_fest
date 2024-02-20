// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tutorial_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tutorial_interfaces__msg__Board __attribute__((deprecated))
#else
# define DEPRECATED__tutorial_interfaces__msg__Board __declspec(deprecated)
#endif

namespace tutorial_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Board_
{
  using Type = Board_<ContainerAllocator>;

  explicit Board_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key_up = 0l;
      this->key_down = 0l;
      this->key_left = 0l;
      this->key_right = 0l;
      this->key_left_shift = 0l;
      this->key_left_ctrl = 0l;
      this->w = 0l;
      this->a = 0l;
      this->s = 0l;
      this->d = 0l;
      this->q = 0l;
      this->e = 0l;
      this->c = 0l;
      this->x = 0l;
      this->z = 0l;
      this->j = 0l;
    }
  }

  explicit Board_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key_up = 0l;
      this->key_down = 0l;
      this->key_left = 0l;
      this->key_right = 0l;
      this->key_left_shift = 0l;
      this->key_left_ctrl = 0l;
      this->w = 0l;
      this->a = 0l;
      this->s = 0l;
      this->d = 0l;
      this->q = 0l;
      this->e = 0l;
      this->c = 0l;
      this->x = 0l;
      this->z = 0l;
      this->j = 0l;
    }
  }

  // field types and members
  using _key_up_type =
    int32_t;
  _key_up_type key_up;
  using _key_down_type =
    int32_t;
  _key_down_type key_down;
  using _key_left_type =
    int32_t;
  _key_left_type key_left;
  using _key_right_type =
    int32_t;
  _key_right_type key_right;
  using _key_left_shift_type =
    int32_t;
  _key_left_shift_type key_left_shift;
  using _key_left_ctrl_type =
    int32_t;
  _key_left_ctrl_type key_left_ctrl;
  using _w_type =
    int32_t;
  _w_type w;
  using _a_type =
    int32_t;
  _a_type a;
  using _s_type =
    int32_t;
  _s_type s;
  using _d_type =
    int32_t;
  _d_type d;
  using _q_type =
    int32_t;
  _q_type q;
  using _e_type =
    int32_t;
  _e_type e;
  using _c_type =
    int32_t;
  _c_type c;
  using _x_type =
    int32_t;
  _x_type x;
  using _z_type =
    int32_t;
  _z_type z;
  using _j_type =
    int32_t;
  _j_type j;

  // setters for named parameter idiom
  Type & set__key_up(
    const int32_t & _arg)
  {
    this->key_up = _arg;
    return *this;
  }
  Type & set__key_down(
    const int32_t & _arg)
  {
    this->key_down = _arg;
    return *this;
  }
  Type & set__key_left(
    const int32_t & _arg)
  {
    this->key_left = _arg;
    return *this;
  }
  Type & set__key_right(
    const int32_t & _arg)
  {
    this->key_right = _arg;
    return *this;
  }
  Type & set__key_left_shift(
    const int32_t & _arg)
  {
    this->key_left_shift = _arg;
    return *this;
  }
  Type & set__key_left_ctrl(
    const int32_t & _arg)
  {
    this->key_left_ctrl = _arg;
    return *this;
  }
  Type & set__w(
    const int32_t & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__a(
    const int32_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__s(
    const int32_t & _arg)
  {
    this->s = _arg;
    return *this;
  }
  Type & set__d(
    const int32_t & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__q(
    const int32_t & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__e(
    const int32_t & _arg)
  {
    this->e = _arg;
    return *this;
  }
  Type & set__c(
    const int32_t & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__x(
    const int32_t & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__z(
    const int32_t & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__j(
    const int32_t & _arg)
  {
    this->j = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tutorial_interfaces::msg::Board_<ContainerAllocator> *;
  using ConstRawPtr =
    const tutorial_interfaces::msg::Board_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::msg::Board_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tutorial_interfaces::msg::Board_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tutorial_interfaces__msg__Board
    std::shared_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tutorial_interfaces__msg__Board
    std::shared_ptr<tutorial_interfaces::msg::Board_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Board_ & other) const
  {
    if (this->key_up != other.key_up) {
      return false;
    }
    if (this->key_down != other.key_down) {
      return false;
    }
    if (this->key_left != other.key_left) {
      return false;
    }
    if (this->key_right != other.key_right) {
      return false;
    }
    if (this->key_left_shift != other.key_left_shift) {
      return false;
    }
    if (this->key_left_ctrl != other.key_left_ctrl) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->s != other.s) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->j != other.j) {
      return false;
    }
    return true;
  }
  bool operator!=(const Board_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Board_

// alias to use template instance with default allocator
using Board =
  tutorial_interfaces::msg::Board_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_HPP_
