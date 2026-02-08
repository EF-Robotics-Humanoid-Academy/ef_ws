// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from g1_interface:srv/G1Volume.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_VOLUME__BUILDER_HPP_
#define G1_INTERFACE__SRV__DETAIL__G1_VOLUME__BUILDER_HPP_

#include "g1_interface/srv/detail/g1_volume__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace g1_interface
{

namespace srv
{

namespace builder
{

class Init_G1Volume_Request_volume_level
{
public:
  Init_G1Volume_Request_volume_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::g1_interface::srv::G1Volume_Request volume_level(::g1_interface::srv::G1Volume_Request::_volume_level_type arg)
  {
    msg_.volume_level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::g1_interface::srv::G1Volume_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::g1_interface::srv::G1Volume_Request>()
{
  return g1_interface::srv::builder::Init_G1Volume_Request_volume_level();
}

}  // namespace g1_interface


namespace g1_interface
{

namespace srv
{

namespace builder
{

class Init_G1Volume_Response_reason
{
public:
  explicit Init_G1Volume_Response_reason(::g1_interface::srv::G1Volume_Response & msg)
  : msg_(msg)
  {}
  ::g1_interface::srv::G1Volume_Response reason(::g1_interface::srv::G1Volume_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::g1_interface::srv::G1Volume_Response msg_;
};

class Init_G1Volume_Response_success
{
public:
  Init_G1Volume_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_G1Volume_Response_reason success(::g1_interface::srv::G1Volume_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_G1Volume_Response_reason(msg_);
  }

private:
  ::g1_interface::srv::G1Volume_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::g1_interface::srv::G1Volume_Response>()
{
  return g1_interface::srv::builder::Init_G1Volume_Response_success();
}

}  // namespace g1_interface

#endif  // G1_INTERFACE__SRV__DETAIL__G1_VOLUME__BUILDER_HPP_
