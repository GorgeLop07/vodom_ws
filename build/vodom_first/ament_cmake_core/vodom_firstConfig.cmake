# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vodom_first_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vodom_first_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vodom_first_FOUND FALSE)
  elseif(NOT vodom_first_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vodom_first_FOUND FALSE)
  endif()
  return()
endif()
set(_vodom_first_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vodom_first_FIND_QUIETLY)
  message(STATUS "Found vodom_first: 0.0.0 (${vodom_first_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vodom_first' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vodom_first_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vodom_first_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vodom_first_DIR}/${_extra}")
endforeach()
