# Automatically generated from BUILD in icon/interprocess

add_library(icon_shared_memory_icon_interprocess_binary_futex STATIC
  "${CMAKE_CURRENT_LIST_DIR}/binary_futex.cc"
  "${CMAKE_CURRENT_LIST_DIR}/binary_futex.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_binary_futex PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_binary_futex PUBLIC
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_strerror
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_interprocess_binary_futex
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/binary_futex.h"
        DESTINATION "include/icon/interprocess"
)
