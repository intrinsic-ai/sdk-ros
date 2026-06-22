# Automatically generated from BUILD in icon/interprocess/shared_memory_lockstep

add_library(icon_shared_memory_icon_interprocess_shared_memory_lockstep STATIC
  "${CMAKE_CURRENT_LIST_DIR}/shared_memory_lockstep.cc"
  "${CMAKE_CURRENT_LIST_DIR}/shared_memory_lockstep.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_lockstep PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_lockstep PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  icon_shared_memory_util_thread_lockstep
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_lockstep
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/shared_memory_lockstep.h"
        DESTINATION "include/icon/interprocess/shared_memory_lockstep"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_lockstep_shared_memory_lockstep_test
    "${CMAKE_CURRENT_LIST_DIR}/shared_memory_lockstep_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_lockstep_shared_memory_lockstep_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_lockstep_shared_memory_lockstep_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_lockstep
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    icon_shared_memory_util_thread
    icon_shared_memory_util_thread_lockstep
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_lockstep_shared_memory_lockstep_test)
endif()
