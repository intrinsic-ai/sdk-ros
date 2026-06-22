# Automatically generated from BUILD in icon/interprocess/shared_memory_manager/testing

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name STATIC
  "${CMAKE_CURRENT_LIST_DIR}/unique_segment_name.cc"
  "${CMAKE_CURRENT_LIST_DIR}/unique_segment_name.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/unique_segment_name.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager/testing"
)
