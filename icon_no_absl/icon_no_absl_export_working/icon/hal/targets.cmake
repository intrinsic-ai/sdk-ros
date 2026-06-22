# Automatically generated from BUILD in icon/hal

add_library(icon_shared_memory_icon_hal_hardware_interface_traits INTERFACE)
target_sources(icon_shared_memory_icon_hal_hardware_interface_traits PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_traits.h"
)
target_include_directories(icon_shared_memory_icon_hal_hardware_interface_traits INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
install(TARGETS icon_shared_memory_icon_hal_hardware_interface_traits
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_traits.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::hardware_interface_traits ALIAS icon_shared_memory_icon_hal_hardware_interface_traits)

add_library(icon_shared_memory_icon_hal_hardware_interface_handle INTERFACE)
target_sources(icon_shared_memory_icon_hal_hardware_interface_handle PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_handle.h"
)
target_include_directories(icon_shared_memory_icon_hal_hardware_interface_handle INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_hardware_interface_handle INTERFACE
  icon_shared_memory_icon_hal_icon_state_register
  icon_state_fbs_cc
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_utils_current_cycle
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_time
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_hardware_interface_handle
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_handle.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::hardware_interface_handle ALIAS icon_shared_memory_icon_hal_hardware_interface_handle)

add_library(icon_shared_memory_icon_hal_hardware_interface_registry STATIC
  "${CMAKE_CURRENT_LIST_DIR}/get_hardware_interface.h"
  "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_registry.cc"
  "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_registry.h"
)
target_include_directories(icon_shared_memory_icon_hal_hardware_interface_registry PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_hardware_interface_registry PUBLIC
  icon_shared_memory_icon_hal_hardware_interface_handle
  icon_shared_memory_icon_hal_hardware_interface_traits
  icon_shared_memory_icon_hal_icon_state_register
  icon_state_fbs_cc
  segment_info_fbs_cc
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_icon_interprocess_shared_memory_manager_domain_socket_utils
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
  icon_shared_memory_icon_interprocess_shared_memory_manager_segment_info_utils
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_realtime_guard
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_hardware_interface_registry
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/get_hardware_interface.h"
        "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_registry.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::hardware_interface_registry ALIAS icon_shared_memory_icon_hal_hardware_interface_registry)

add_library(icon_shared_memory_icon_hal_hardware_module_interface INTERFACE)
target_sources(icon_shared_memory_icon_hal_hardware_module_interface PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/hardware_module_interface.h"
)
target_include_directories(icon_shared_memory_icon_hal_hardware_module_interface INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_hardware_module_interface INTERFACE
  icon_shared_memory_icon_control_realtime_clock_interface
  icon_shared_memory_icon_utils_status
)
install(TARGETS icon_shared_memory_icon_hal_hardware_module_interface
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/hardware_module_interface.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::hardware_module_interface ALIAS icon_shared_memory_icon_hal_hardware_module_interface)

add_library(icon_shared_memory_icon_hal_icon_state_register INTERFACE)
target_sources(icon_shared_memory_icon_hal_icon_state_register PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/icon_state_register.h"
)
target_include_directories(icon_shared_memory_icon_hal_icon_state_register INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_icon_state_register INTERFACE
  icon_shared_memory_icon_hal_hardware_interface_traits
  icon_state_fbs_cc
  icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils
)
install(TARGETS icon_shared_memory_icon_hal_icon_state_register
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/icon_state_register.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::icon_state_register ALIAS icon_shared_memory_icon_hal_icon_state_register)

add_library(icon_shared_memory_icon_hal_realtime_clock STATIC
  "${CMAKE_CURRENT_LIST_DIR}/realtime_clock.cc"
  "${CMAKE_CURRENT_LIST_DIR}/realtime_clock.h"
)
target_include_directories(icon_shared_memory_icon_hal_realtime_clock PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_realtime_clock PUBLIC
  icon_shared_memory_icon_control_realtime_clock_interface
  icon_shared_memory_icon_interprocess_shared_memory_lockstep
  icon_shared_memory_icon_interprocess_shared_memory_manager
  icon_shared_memory_icon_interprocess_shared_memory_manager_memory_segment
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_time
  icon_shared_memory_util_thread_lockstep
)
install(TARGETS icon_shared_memory_icon_hal_realtime_clock
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/realtime_clock.h"
        DESTINATION "include/icon/hal"
)
add_library(icon::hal::realtime_clock ALIAS icon_shared_memory_icon_hal_realtime_clock)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_hal_hardware_interface_registry_test
    "${CMAKE_CURRENT_LIST_DIR}/hardware_interface_registry_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_hal_hardware_interface_registry_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_hal_hardware_interface_registry_test PRIVATE
    icon_shared_memory_icon_hal_hardware_interface_handle
    icon_shared_memory_icon_hal_hardware_interface_registry
    icon_shared_memory_icon_hal_hardware_interface_traits
    icon_shared_memory_icon_hal_icon_state_register
    icon_state_fbs_cc
    joint_limits_fbs_cc
    icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_current_cycle
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_icon_utils_time
    flatbuffers::flatbuffers
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_hal_hardware_interface_registry_test)
endif()

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_hal_get_hardware_interface_test
    "${CMAKE_CURRENT_LIST_DIR}/get_hardware_interface_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_hal_get_hardware_interface_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_hal_get_hardware_interface_test PRIVATE
    icon_shared_memory_icon_hal_hardware_interface_registry
    icon_shared_memory_icon_hal_icon_state_register
    icon_state_fbs_cc
    joint_limits_fbs_cc
    segment_info_fbs_cc
    icon_shared_memory_icon_flatbuffers_flatbuffer_utils
    icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils
    icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
    icon_shared_memory_icon_interprocess_shared_memory_manager_segment_header
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    flatbuffers::flatbuffers
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_hal_get_hardware_interface_test)
endif()
