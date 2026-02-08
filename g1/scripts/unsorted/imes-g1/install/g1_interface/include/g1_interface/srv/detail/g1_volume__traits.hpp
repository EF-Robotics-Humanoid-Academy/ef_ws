// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from g1_interface:srv/G1Volume.idl
// generated code does not contain a copyright notice

#ifndef G1_INTERFACE__SRV__DETAIL__G1_VOLUME__TRAITS_HPP_
#define G1_INTERFACE__SRV__DETAIL__G1_VOLUME__TRAITS_HPP_

#include "g1_interface/srv/detail/g1_volume__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<g1_interface::srv::G1Volume_Request>()
{
  return "g1_interface::srv::G1Volume_Request";
}

template<>
inline const char * name<g1_interface::srv::G1Volume_Request>()
{
  return "g1_interface/srv/G1Volume_Request";
}

template<>
struct has_fixed_size<g1_interface::srv::G1Volume_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<g1_interface::srv::G1Volume_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<g1_interface::srv::G1Volume_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<g1_interface::srv::G1Volume_Response>()
{
  return "g1_interface::srv::G1Volume_Response";
}

template<>
inline const char * name<g1_interface::srv::G1Volume_Response>()
{
  return "g1_interface/srv/G1Volume_Response";
}

template<>
struct has_fixed_size<g1_interface::srv::G1Volume_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<g1_interface::srv::G1Volume_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<g1_interface::srv::G1Volume_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<g1_interface::srv::G1Volume>()
{
  return "g1_interface::srv::G1Volume";
}

template<>
inline const char * name<g1_interface::srv::G1Volume>()
{
  return "g1_interface/srv/G1Volume";
}

template<>
struct has_fixed_size<g1_interface::srv::G1Volume>
  : std::integral_constant<
    bool,
    has_fixed_size<g1_interface::srv::G1Volume_Request>::value &&
    has_fixed_size<g1_interface::srv::G1Volume_Response>::value
  >
{
};

template<>
struct has_bounded_size<g1_interface::srv::G1Volume>
  : std::integral_constant<
    bool,
    has_bounded_size<g1_interface::srv::G1Volume_Request>::value &&
    has_bounded_size<g1_interface::srv::G1Volume_Response>::value
  >
{
};

template<>
struct is_service<g1_interface::srv::G1Volume>
  : std::true_type
{
};

template<>
struct is_service_request<g1_interface::srv::G1Volume_Request>
  : std::true_type
{
};

template<>
struct is_service_response<g1_interface::srv::G1Volume_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // G1_INTERFACE__SRV__DETAIL__G1_VOLUME__TRAITS_HPP_
