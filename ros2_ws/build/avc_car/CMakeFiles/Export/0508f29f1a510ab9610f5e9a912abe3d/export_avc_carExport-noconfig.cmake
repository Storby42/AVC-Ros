#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "avc_car::avc_car" for configuration ""
set_property(TARGET avc_car::avc_car APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(avc_car::avc_car PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libavc_car.so"
  IMPORTED_SONAME_NOCONFIG "libavc_car.so"
  )

list(APPEND _cmake_import_check_targets avc_car::avc_car )
list(APPEND _cmake_import_check_files_for_avc_car::avc_car "${_IMPORT_PREFIX}/lib/libavc_car.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
