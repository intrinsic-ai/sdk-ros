# Automatically generated from BUILD in kinematics/types

add_library(icon_shared_memory_kinematics_types_joint_limits STATIC
  "${CMAKE_CURRENT_LIST_DIR}/joint_limits.cc"
  "${CMAKE_CURRENT_LIST_DIR}/joint_limits.h"
)
target_include_directories(icon_shared_memory_kinematics_types_joint_limits PUBLIC
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_kinematics_types_joint_limits PUBLIC
  icon_shared_memory_eigenmath
  icon_shared_memory_icon_utils_log
  icon_shared_memory_icon_utils_status
  tl::expected
)
install(TARGETS icon_shared_memory_kinematics_types_joint_limits
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/joint_limits.h"
        DESTINATION "include/kinematics/types"
)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_kinematics_types_joint_limits_test
    "${CMAKE_CURRENT_LIST_DIR}/joint_limits_test.cc"
  )
  target_include_directories(icon_shared_memory_kinematics_types_joint_limits_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_kinematics_types_joint_limits_test PRIVATE
    icon_shared_memory_kinematics_types_joint_limits
    icon_shared_memory_eigenmath
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_kinematics_types_joint_limits_test)
endif()
