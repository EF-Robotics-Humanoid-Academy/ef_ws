// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from g1_interface:srv/G1Volume.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_VOLUME__STRUCT_HPP_
#define G1_INTERFACE__SRV__DETAIL__G1_VOLUME__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__g1_interface__srv__G1Volume_Request __attribute__((deprecated))
#else
# define DEPRECATED__g1_interface__srv__G1Volume_Request __declspec(deprecated)
#endif

namespace g1_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct G1Volume_Request_
{
  using Type = G1Volume_Request_<ContainerAllocator>;

  explicit G1Volume_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->volume_level = 0l;
    }
  }

  explicit G1Volume_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->volume_level = 0l;
    }
  }

  // field types and members
  using _volume_level_type =
    int32_t;
  _volume_level_type volume_level;

  // setters for named parameter idiom
  Type & set__volume_level(
    const int32_t & _arg)
  {
    this->volume_level = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    g1_interface::srv::G1Volume_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const g1_interface::srv::G1Volume_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      g1_interface::srv::G1Volume_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      g1_interface::srv::G1Volume_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__g1_interface__srv__G1Volume_Request
    std::shared_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__g1_interface__srv__G1Volume_Request
    std::shared_ptr<g1_interface::srv::G1Volume_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const G1Volume_Request_ & other) const
  {
    if (this->volume_level != other.volume_level) {
      return false;
    }
    return true;
  }
  bool operator!=(const G1Volume_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct G1Volume_Request_

// alias to use template instance with default allocator
using G1Volume_Request =
  g1_interface::srv::G1Volume_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace g1_interface


#ifndef _WIN32
# define DEPRECATED__g1_interface__srv__G1Volume_Response __attribute__((deprecated))
#else
# define DEPRECATED__g1_interface__srv__G1Volume_Response __declspec(deprecated)
#endif

namespace g1_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct G1Volume_Response_
{
  using Type = G1Volume_Response_<ContainerAllocator>;

  explicit G1Volume_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reason = "";
    }
  }

  explicit G1Volume_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reason = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    g1_interface::srv::G1Volume_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const g1_interface::srv::G1Volume_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      g1_interface::srv::G1Volume_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      g1_interface::srv::G1Volume_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__g1_interface__srv__G1Volume_Response
    std::shared_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__g1_interface__srv__G1Volume_Response
    std::shared_ptr<g1_interface::srv::G1Volume_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const G1Volume_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const G1Volume_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct G1Volume_Response_

// alias to use template instance with default allocator
using G1Volume_Response =
  g1_interface::srv::G1Volume_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace g1_interface

namespace g1_interface
{

namespace srv
{

struct G1Volume
{
  using Request = g1_interface::srv::G1Volume_Request;
  using Response = g1_interface::srv::G1Volume_Response;
};

}  // namespace srv

}  // namespace g1_interface

#endif  // G1_INTERFACE__SRV__DETAIL__G1_VOLUME__STRUCT_HPP_
