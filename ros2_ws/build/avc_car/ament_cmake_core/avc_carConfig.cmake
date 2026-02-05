# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_avc_car_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED avc_car_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(avc_car_FOUND FALSE)
  elseif(NOT avc_car_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(avc_car_FOUND FALSE)
  endif()
  return()
endif()
set(_avc_car_CONFIG_INCLUDED TRUE)

# output package information
if(NOT avc_car_FIND_QUIETLY)
  message(STATUS "Found avc_car: 0.0.0 (${avc_car_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'avc_car' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT avc_car_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(avc_car_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${avc_car_DIR}/${_extra}")
endforeach()
