# Automatically generated from BUILD in util/thread

add_library(icon_shared_memory_util_thread STATIC
  "${CMAKE_CURRENT_LIST_DIR}/thread.cc"
  "${CMAKE_CURRENT_LIST_DIR}/thread.h"
)
target_include_directories(icon_shared_memory_util_thread PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_util_thread PUBLIC
  icon_shared_memory_util_thread_stop_token
  icon_shared_memory_icon_utils_realtime_guard
  icon_shared_memory_icon_utils_status
)
install(TARGETS icon_shared_memory_util_thread
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/thread.h"
        DESTINATION "include/util/thread"
)
add_library(util::thread ALIAS icon_shared_memory_util_thread)

add_library(icon_shared_memory_util_thread_stop_token INTERFACE)
target_sources(icon_shared_memory_util_thread_stop_token PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/stop_token.h"
)
target_include_directories(icon_shared_memory_util_thread_stop_token INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_util_thread_stop_token INTERFACE
  icon_shared_memory_icon_utils_attributes
)
install(TARGETS icon_shared_memory_util_thread_stop_token
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/stop_token.h"
        DESTINATION "include/util/thread"
)
add_library(util::thread::stop_token ALIAS icon_shared_memory_util_thread_stop_token)

add_library(icon_shared_memory_util_thread_lockstep STATIC
  "${CMAKE_CURRENT_LIST_DIR}/lockstep.cc"
  "${CMAKE_CURRENT_LIST_DIR}/lockstep.h"
)
target_include_directories(icon_shared_memory_util_thread_lockstep PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_util_thread_lockstep PUBLIC
  icon_shared_memory_icon_interprocess_binary_futex
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_util_thread_lockstep
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/lockstep.h"
        DESTINATION "include/util/thread"
)
add_library(util::thread::lockstep ALIAS icon_shared_memory_util_thread_lockstep)
