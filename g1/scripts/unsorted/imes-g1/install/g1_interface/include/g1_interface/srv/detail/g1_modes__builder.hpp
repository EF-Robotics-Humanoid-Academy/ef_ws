// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from g1_interface:srv/G1Modes.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_MODES__BUILDER_HPP_
#define G1_INTERFACE__SRV__DETAIL__G1_MODES__BUILDER_HPP_

#include "g1_interface/srv/detail/g1_modes__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace g1_interface
{

namespace srv
{

namespace builder
{

class Init_G1Modes_Request_request_data
{
public:
  Init_G1Modes_Request_request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::g1_interface::srv::G1Modes_Request request_data(::g1_interface::srv::G1Modes_Request::_request_data_type arg)
  {
    msg_.request_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::g1_interface::srv::G1Modes_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::g1_interface::srv::G1Modes_Request>()
{
  return g1_interface::srv::builder::Init_G1Modes_Request_request_data();
}

}  // namespace g1_interface


namespace g1_interface
{

namespace srv
{

namespace builder
{

class Init_G1Modes_Response_reason
{
public:
  explicit Init_G1Modes_Response_reason(::g1_interface::srv::G1Modes_Response & msg)
  : msg_(msg)
  {}
  ::g1_interface::srv::G1Modes_Response reason(::g1_interface::srv::G1Modes_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::g1_interface::srv::G1Modes_Response msg_;
};

class Init_G1Modes_Response_success
{
public:
  Init_G1Modes_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_G1Modes_Response_reason success(::g1_interface::srv::G1Modes_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_G1Modes_Response_reason(msg_);
  }

private:
  ::g1_interface::srv::G1Modes_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::g1_interface::srv::G1Modes_Response>()
{
  return g1_interface::srv::builder::Init_G1Modes_Response_success();
}

}  // namespace g1_interface

#endif  // G1_INTERFACE__SRV__DETAIL__G1_MODES__BUILDER_HPP_
