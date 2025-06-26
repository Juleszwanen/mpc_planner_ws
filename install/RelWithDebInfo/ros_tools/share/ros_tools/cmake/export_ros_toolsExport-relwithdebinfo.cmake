#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros_tools::ros_tools" for configuration "RelWithDebInfo"
set_property(TARGET ros_tools::ros_tools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ros_tools::ros_tools PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libros_tools.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libros_tools.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros_tools::ros_tools )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros_tools::ros_tools "${_IMPORT_PREFIX}/lib/libros_tools.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
