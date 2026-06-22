# Automatically generated from BUILD in icon/hal/interfaces

add_library(icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/hardware_module_state_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/hardware_module_state_utils.h"
)
target_include_directories(icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_utils_attributes
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_interfaces_hardware_module_state_fbs_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/hardware_module_state_utils.h"
        DESTINATION "include/icon/hal/interfaces"
)

add_library(icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/icon_state_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/icon_state_utils.h"
)
target_include_directories(icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_utils_attributes
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_interfaces_icon_state_fbs_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/icon_state_utils.h"
        DESTINATION "include/icon/hal/interfaces"
)

add_library(icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/joint_limits_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/joint_limits_utils.h"
)
target_include_directories(icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_hal_hardware_interface_handle
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  icon_shared_memory_kinematics_types_joint_limits
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/joint_limits_utils.h"
        DESTINATION "include/icon/hal/interfaces"
)

add_library(icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/joint_command_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/joint_command_utils.h"
)
target_include_directories(icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_flatbuffers_flatbuffer_utils
  icon_shared_memory_icon_utils_attributes
  icon_shared_memory_icon_utils_status
  icon_shared_memory_icon_utils_status_and_expected_macros
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_interfaces_joint_command_fbs_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/joint_command_utils.h"
        DESTINATION "include/icon/hal/interfaces"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_hal_interfaces_joint_limits_utils_test
    "${CMAKE_CURRENT_LIST_DIR}/joint_limits_utils_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_hal_interfaces_joint_limits_utils_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_hal_interfaces_joint_limits_utils_test PRIVATE
    icon_shared_memory_icon_hal_interfaces_joint_limits_fbs_utils
    icon_shared_memory_external_fbs_cc
    icon_shared_memory_icon_hal_hardware_interface_handle
    icon_shared_memory_icon_hal_hardware_interface_registry
    icon_shared_memory_icon_hal_hardware_interface_traits
    icon_shared_memory_icon_interprocess_shared_memory_manager
    icon_shared_memory_icon_interprocess_shared_memory_manager_testing_unique_segment_name
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    icon_shared_memory_kinematics_types_joint_limits
    flatbuffers::flatbuffers
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_hal_interfaces_joint_limits_utils_test)
endif()

add_library(icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils STATIC
  "${CMAKE_CURRENT_LIST_DIR}/joint_state_utils.cc"
  "${CMAKE_CURRENT_LIST_DIR}/joint_state_utils.h"
)
target_include_directories(icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils PUBLIC
  icon_shared_memory_external_fbs_cc
  icon_shared_memory_icon_utils_attributes
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_hal_interfaces_joint_state_fbs_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/joint_state_utils.h"
        DESTINATION "include/icon/hal/interfaces"
)
