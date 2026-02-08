# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_g1_moveit_29dof_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED g1_moveit_29dof_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(g1_moveit_29dof_FOUND FALSE)
  elseif(NOT g1_moveit_29dof_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(g1_moveit_29dof_FOUND FALSE)
  endif()
  return()
endif()
set(_g1_moveit_29dof_CONFIG_INCLUDED TRUE)

# output package information
if(NOT g1_moveit_29dof_FIND_QUIETLY)
  message(STATUS "Found g1_moveit_29dof: 2.1.0 (${g1_moveit_29dof_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'g1_moveit_29dof' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${g1_moveit_29dof_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(g1_moveit_29dof_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${g1_moveit_29dof_DIR}/${_extra}")
endforeach()
