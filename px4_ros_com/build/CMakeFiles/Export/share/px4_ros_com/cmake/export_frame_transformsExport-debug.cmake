#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "px4_ros_com::frame_transforms" for configuration "Debug"
set_property(TARGET px4_ros_com::frame_transforms APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(px4_ros_com::frame_transforms PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libframe_transforms.so"
  IMPORTED_SONAME_DEBUG "libframe_transforms.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS px4_ros_com::frame_transforms )
list(APPEND _IMPORT_CHECK_FILES_FOR_px4_ros_com::frame_transforms "${_IMPORT_PREFIX}/lib/libframe_transforms.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
