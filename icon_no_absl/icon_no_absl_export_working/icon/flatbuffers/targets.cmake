# Automatically generated from BUILD in icon/flatbuffers

add_library(icon_shared_memory_icon_flatbuffers_flatbuffer_utils INTERFACE)
target_sources(icon_shared_memory_icon_flatbuffers_flatbuffer_utils PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/flatbuffer_utils.h"
)
target_include_directories(icon_shared_memory_icon_flatbuffers_flatbuffer_utils INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_flatbuffers_flatbuffer_utils INTERFACE
  icon_shared_memory_icon_utils_status
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_flatbuffers_flatbuffer_utils
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/flatbuffer_utils.h"
        DESTINATION "include/icon/flatbuffers"
)
add_library(icon::flatbuffers::flatbuffer_utils ALIAS icon_shared_memory_icon_flatbuffers_flatbuffer_utils)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_flatbuffers_flatbuffer_utils_test
    "${CMAKE_CURRENT_LIST_DIR}/flatbuffer_utils_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_flatbuffers_flatbuffer_utils_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_flatbuffers_flatbuffer_utils_test PRIVATE
    icon_shared_memory_icon_flatbuffers_flatbuffer_utils
    transform_types_fbs_cc
    segment_info_fbs_cc
    icon_shared_memory_icon_utils_status
    flatbuffers::flatbuffers
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_flatbuffers_flatbuffer_utils_test)
endif()

add_library(icon_shared_memory_icon_flatbuffers_fixed_string INTERFACE)
target_sources(icon_shared_memory_icon_flatbuffers_fixed_string PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/fixed_string.h"
)
target_include_directories(icon_shared_memory_icon_flatbuffers_fixed_string INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_icon_flatbuffers_fixed_string INTERFACE
  icon_shared_memory_icon_utils_status
  flatbuffers::flatbuffers
  tl::expected
)
install(TARGETS icon_shared_memory_icon_flatbuffers_fixed_string
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/fixed_string.h"
        DESTINATION "include/icon/flatbuffers"
)
add_library(icon::flatbuffers::fixed_string ALIAS icon_shared_memory_icon_flatbuffers_fixed_string)

install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/fixed_string_test.fbs"
        DESTINATION "include/icon/flatbuffers"
)

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/icon/flatbuffers/fixed_string_test.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/icon/flatbuffers"
          -I "${INSRC_ROOT}"
          "${CMAKE_CURRENT_LIST_DIR}/fixed_string_test.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/fixed_string_test.fbs"
  COMMENT "Generating C++ Flatbuffers headers for fixed_string_test.fbs"
)
add_library(icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc INTERFACE)
target_include_directories(icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/icon/flatbuffers/fixed_string_test.fbs.h"
)
target_link_libraries(icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc
        EXPORT icon_shared_memoryTargets
)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/icon/flatbuffers/fixed_string_test.fbs.h"
        DESTINATION "include/icon/flatbuffers"
)
add_library(icon::flatbuffers::fixed_string_test_fbs_cc ALIAS icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc)

if(BUILD_TESTING)
  add_executable(icon_shared_memory_icon_flatbuffers_fixed_string_test
    "${CMAKE_CURRENT_LIST_DIR}/fixed_string_test.cc"
  )
  target_include_directories(icon_shared_memory_icon_flatbuffers_fixed_string_test PRIVATE "${INSRC_ROOT}")
  target_link_libraries(icon_shared_memory_icon_flatbuffers_fixed_string_test PRIVATE
    icon_shared_memory_icon_flatbuffers_fixed_string
    icon_shared_memory_icon_flatbuffers_fixed_string_test_fbs_cc
    icon_shared_memory_icon_utils_status
    icon_shared_memory_icon_utils_status_and_expected_test_macros
    flatbuffers::flatbuffers
    GTest::gtest
  )
  gtest_add_tests(TARGET icon_shared_memory_icon_flatbuffers_fixed_string_test)
endif()
