# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cychuankou_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cychuankou_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cychuankou_FOUND FALSE)
  elseif(NOT cychuankou_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cychuankou_FOUND FALSE)
  endif()
  return()
endif()
set(_cychuankou_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cychuankou_FIND_QUIETLY)
  message(STATUS "Found cychuankou: 0.0.0 (${cychuankou_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cychuankou' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cychuankou_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cychuankou_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cychuankou_DIR}/${_extra}")
endforeach()
