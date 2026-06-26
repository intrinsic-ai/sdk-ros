# Re-define imw_zenoh imported target
if(NOT TARGET imw_zenoh)
  add_library(imw_zenoh SHARED IMPORTED)
  set_target_properties(imw_zenoh PROPERTIES
    IMPORTED_LOCATION "${_prefix}/lib/libimw_zenoh.so"
  )
endif()

# Append imw_zenoh to interface link libraries of intrinsic_sdk_cmake::intrinsic_sdk_cmake
if(TARGET intrinsic_sdk_cmake::intrinsic_sdk_cmake)
  set_property(TARGET intrinsic_sdk_cmake::intrinsic_sdk_cmake APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES imw_zenoh
  )
endif()
