# Automatically generated from BUILD in icon/control

add_library(icon_shared_memory_icon_control_realtime_clock_interface STATIC
  "${CMAKE_CURRENT_LIST_DIR}/realtime_clock_interface.cc"
  "${CMAKE_CURRENT_LIST_DIR}/realtime_clock_interface.h"
)
target_include_directories(icon_shared_memory_icon_control_realtime_clock_interface PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_control_realtime_clock_interface PUBLIC
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_control_realtime_clock_interface
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/realtime_clock_interface.h"
        DESTINATION "include/icon/control"
)
