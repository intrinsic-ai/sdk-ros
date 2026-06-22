# Automatically generated from BUILD in icon/interprocess/remote_trigger

add_library(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server STATIC
  "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_constants.h"
  "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_server.cc"
  "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_server.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server PUBLIC
  icon_shared_memory_icon_interprocess_binary_futex
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  icon_shared_memory_icon_utils_time
  icon_shared_memory_util_thread
  tl::expected
)
install(TARGETS icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_constants.h"
        "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_server.h"
        DESTINATION "include/icon/interprocess/remote_trigger"
)

add_library(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common INTERFACE)
target_sources(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_test_common.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common INTERFACE
  icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server
)
install(TARGETS icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_test_common.h"
        DESTINATION "include/icon/interprocess/remote_trigger"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server_test
    "${CMAKE_CURRENT_LIST_DIR}/remote_trigger_server_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server_test PRIVATE
    icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server
    icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_test_common
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_util_thread
    GTest::gtest
    tl::expected
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_remote_trigger_remote_trigger_server_test)
endif()
