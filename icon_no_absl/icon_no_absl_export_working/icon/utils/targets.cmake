# Automatically generated from BUILD in icon/utils

add_library(icon_shared_memory_icon_utils_attributes INTERFACE)
target_sources(icon_shared_memory_icon_utils_attributes PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/attributes.h"
)
target_include_directories(icon_shared_memory_icon_utils_attributes INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_attributes
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/attributes.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::attributes ALIAS icon_shared_memory_icon_utils_attributes)

add_library(icon_shared_memory_icon_utils_cleanup INTERFACE)
target_sources(icon_shared_memory_icon_utils_cleanup PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/cleanup.h"
)
target_include_directories(icon_shared_memory_icon_utils_cleanup INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_utils_cleanup INTERFACE
  icon_shared_memory_icon_utils_attributes
)
install(TARGETS icon_shared_memory_icon_utils_cleanup
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/cleanup.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::cleanup ALIAS icon_shared_memory_icon_utils_cleanup)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_cleanup_test
    "${CMAKE_CURRENT_LIST_DIR}/cleanup_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_cleanup_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_cleanup_test PRIVATE
    icon_shared_memory_icon_utils_cleanup
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_cleanup_test)
endif()

add_library(icon_shared_memory_icon_utils_current_cycle STATIC
  "${CMAKE_CURRENT_LIST_DIR}/current_cycle.cc"
  "${CMAKE_CURRENT_LIST_DIR}/current_cycle.h"
)
target_include_directories(icon_shared_memory_icon_utils_current_cycle PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_current_cycle
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/current_cycle.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::current_cycle ALIAS icon_shared_memory_icon_utils_current_cycle)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_current_cycle_test
    "${CMAKE_CURRENT_LIST_DIR}/current_cycle_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_current_cycle_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_current_cycle_test PRIVATE
    icon_shared_memory_icon_utils_current_cycle
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_current_cycle_test)
endif()

add_library(icon_shared_memory_icon_utils_format INTERFACE)
target_sources(icon_shared_memory_icon_utils_format PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/format.h"
)
target_include_directories(icon_shared_memory_icon_utils_format INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_format
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/format.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::format ALIAS icon_shared_memory_icon_utils_format)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_format_test
    "${CMAKE_CURRENT_LIST_DIR}/format_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_format_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_format_test PRIVATE
    icon_shared_memory_icon_utils_format
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_format_test)
endif()

add_library(icon_shared_memory_icon_utils_log INTERFACE)
target_sources(icon_shared_memory_icon_utils_log PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/log.h"
)
target_include_directories(icon_shared_memory_icon_utils_log INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_log
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/log.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::log ALIAS icon_shared_memory_icon_utils_log)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_log_test
    "${CMAKE_CURRENT_LIST_DIR}/log_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_log_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_log_test PRIVATE
    icon_shared_memory_icon_utils_log
    icon_shared_memory_icon_utils_mock_log_sink
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_log_test)
endif()

add_library(icon_shared_memory_icon_utils_mock_log_sink INTERFACE)
target_sources(icon_shared_memory_icon_utils_mock_log_sink PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/mock_log_sink.h"
)
target_include_directories(icon_shared_memory_icon_utils_mock_log_sink INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_utils_mock_log_sink INTERFACE
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_log
)
install(TARGETS icon_shared_memory_icon_utils_mock_log_sink
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/mock_log_sink.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::mock_log_sink ALIAS icon_shared_memory_icon_utils_mock_log_sink)

add_library(icon_shared_memory_icon_utils_realtime_guard INTERFACE)
target_sources(icon_shared_memory_icon_utils_realtime_guard PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/realtime_guard.h"
)
target_include_directories(icon_shared_memory_icon_utils_realtime_guard INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_realtime_guard
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/realtime_guard.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::realtime_guard ALIAS icon_shared_memory_icon_utils_realtime_guard)

add_library(icon_shared_memory_icon_utils_status INTERFACE)
target_sources(icon_shared_memory_icon_utils_status PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/status.h"
)
target_include_directories(icon_shared_memory_icon_utils_status INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_utils_status INTERFACE
  icon_shared_memory_icon_utils_attributes
)
install(TARGETS icon_shared_memory_icon_utils_status
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/status.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::status ALIAS icon_shared_memory_icon_utils_status)

add_library(icon_shared_memory_icon_utils_status_and_expected_macros INTERFACE)
target_sources(icon_shared_memory_icon_utils_status_and_expected_macros PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/status_and_expected_macros.h"
)
target_include_directories(icon_shared_memory_icon_utils_status_and_expected_macros INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_utils_status_and_expected_macros INTERFACE
  icon_shared_memory_icon_utils_status
  tl::expected
)
install(TARGETS icon_shared_memory_icon_utils_status_and_expected_macros
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/status_and_expected_macros.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::status_and_expected_macros ALIAS icon_shared_memory_icon_utils_status_and_expected_macros)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_status_and_expected_macros_test
    "${CMAKE_CURRENT_LIST_DIR}/status_and_expected_macros_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_status_and_expected_macros_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_status_and_expected_macros_test PRIVATE
    icon_shared_memory_icon_utils_status_and_expected_macros
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_status_and_expected_macros_test)
endif()

add_library(icon_shared_memory_icon_utils_status_and_expected_test_macros INTERFACE)
target_sources(icon_shared_memory_icon_utils_status_and_expected_test_macros PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/status_and_expected_test_macros.h"
)
target_include_directories(icon_shared_memory_icon_utils_status_and_expected_test_macros INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_utils_status_and_expected_test_macros INTERFACE
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  tl::expected
)
install(TARGETS icon_shared_memory_icon_utils_status_and_expected_test_macros
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/status_and_expected_test_macros.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::status_and_expected_test_macros ALIAS icon_shared_memory_icon_utils_status_and_expected_test_macros)

add_library(icon_shared_memory_icon_utils_strerror INTERFACE)
target_sources(icon_shared_memory_icon_utils_strerror PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/strerror.h"
)
target_include_directories(icon_shared_memory_icon_utils_strerror INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_strerror
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/strerror.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::strerror ALIAS icon_shared_memory_icon_utils_strerror)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_strerror_test
    "${CMAKE_CURRENT_LIST_DIR}/strerror_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_strerror_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_strerror_test PRIVATE
    icon_shared_memory_icon_utils_strerror
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_strerror_test)
endif()

add_library(icon_shared_memory_icon_utils_time STATIC
  "${CMAKE_CURRENT_LIST_DIR}/time.cc"
  "${CMAKE_CURRENT_LIST_DIR}/time.h"
)
target_include_directories(icon_shared_memory_icon_utils_time PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_utils_time
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/time.h"
        DESTINATION "include/icon/utils"
)
add_library(icon::utils::time ALIAS icon_shared_memory_icon_utils_time)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_utils_time_test
    "${CMAKE_CURRENT_LIST_DIR}/time_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_utils_time_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_utils_time_test PRIVATE
    icon_shared_memory_icon_utils_time
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_utils_time_test)
endif()
