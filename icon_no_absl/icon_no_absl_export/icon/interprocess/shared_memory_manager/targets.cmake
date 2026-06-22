# Automatically generated from BUILD in icon/interprocess/shared_memory_manager

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header STATIC
  "${CMAKE_CURRENT_LIST_DIR}/segment_header.cc"
  "${CMAKE_CURRENT_LIST_DIR}/segment_header.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header PUBLIC
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/segment_header.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header_test
    "${CMAKE_CURRENT_LIST_DIR}/segment_header_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
    icon_shared_memory_icon_utils_mock_log_sink
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header_test)
endif()

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/segment_info_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/segment_info_utils.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_flatbuffers_fixed_string
  icon_shared_memory_icon_utils_status
  tl::expected
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/segment_info_utils.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils_test
    "${CMAKE_CURRENT_LIST_DIR}/segment_info_utils_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils
    icon_shared_memory_external_fbs_cc
    icon_shared_memory_icon_flatbuffers_flatbuffer_utils
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils_test)
endif()

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager STATIC
  "${CMAKE_CURRENT_LIST_DIR}/shared_memory_manager.cc"
  "${CMAKE_CURRENT_LIST_DIR}/shared_memory_manager.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_flatbuffers_flatbuffer_utils
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  rt
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/shared_memory_manager.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_shared_memory_manager_test
    "${CMAKE_CURRENT_LIST_DIR}/shared_memory_manager_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_shared_memory_manager_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_shared_memory_manager_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server
    icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_external_fbs_cc
    icon_shared_memory_icon_flatbuffers_flatbuffer_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_mock_log_sink
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_shared_memory_manager_test)
endif()

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment STATIC
  "${CMAKE_CURRENT_LIST_DIR}/memory_segment.cc"
  "${CMAKE_CURRENT_LIST_DIR}/memory_segment.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
  icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
  icon_shared_memory_icon_utils_format
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_strerror
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/memory_segment.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server STATIC
  "${CMAKE_CURRENT_LIST_DIR}/domain_socket_server.cc"
  "${CMAKE_CURRENT_LIST_DIR}/domain_socket_server.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_flatbuffers_fixed_string
  icon_shared_memory_icon_hal_hardware_interface_registry
  icon_shared_memory_icon_utils_cleanup
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  icon_shared_memory_icon_utils_time
  icon_shared_memory_util_thread
  icon_shared_memory_util_thread_stop_token
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/domain_socket_server.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

add_library(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/domain_socket_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/domain_socket_utils.h"
)
target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils PUBLIC
  icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_flatbuffers_flatbuffer_utils
  icon_shared_memory_icon_utils_cleanup
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_strerror
  icon_shared_memory_icon_utils_time
)
install(TARGETS icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/domain_socket_utils.h"
        DESTINATION "include/icon/interprocess/shared_memory_manager"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment_test
    "${CMAKE_CURRENT_LIST_DIR}/memory_segment_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_external_fbs_cc
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment_test)
endif()

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils_test
    "${CMAKE_CURRENT_LIST_DIR}/domain_socket_utils_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
    icon_shared_memory_icon_utils_mock_log_sink
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils_test)
endif()

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server_test
    "${CMAKE_CURRENT_LIST_DIR}/domain_socket_server_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server_test PRIVATE
    icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server
    icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_external_fbs_cc
    icon_shared_memory_icon_flatbuffers_flatbuffer_utils
    icon_shared_memory_icon_hal_hardware_interface_registry
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_cleanup
    icon_shared_memory_icon_utils_log
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_server_test)
endif()
