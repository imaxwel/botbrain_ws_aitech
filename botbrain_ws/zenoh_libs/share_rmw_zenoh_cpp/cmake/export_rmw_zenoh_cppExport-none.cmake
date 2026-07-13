#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmw_zenoh_cpp::rmw_zenoh_cpp" for configuration "None"
set_property(TARGET rmw_zenoh_cpp::rmw_zenoh_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(rmw_zenoh_cpp::rmw_zenoh_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "ament_index_cpp::ament_index_cpp;fastcdr;rcpputils::rcpputils;rcutils::rcutils;rosidl_typesupport_fastrtps_c::rosidl_typesupport_fastrtps_c;rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp;rmw::rmw;tracetools::tracetools"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/librmw_zenoh_cpp.so"
  IMPORTED_SONAME_NONE "librmw_zenoh_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmw_zenoh_cpp::rmw_zenoh_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmw_zenoh_cpp::rmw_zenoh_cpp "${_IMPORT_PREFIX}/lib/librmw_zenoh_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
