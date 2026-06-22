find_package(FlatBuffers REQUIRED)
find_program(FLATC_EXECUTABLE flatc REQUIRED)

if(NOT INSRC_ROOT)
  get_filename_component(INSRC_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

# Target: transform_types_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/flatbuffers/transform_types.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/flatbuffers"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/flatbuffers/transform_types.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/flatbuffers/transform_types.fbs"
  COMMENT "Generating C++ Flatbuffers headers for transform_types.fbs"
)
add_library(transform_types_fbs_cc INTERFACE)
target_include_directories(transform_types_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(transform_types_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/flatbuffers/transform_types.fbs.h"
)
target_link_libraries(transform_types_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS transform_types_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/flatbuffers/transform_types.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/flatbuffers"
)
add_library(icon::flatbuffers::transform_types_fbs_cc ALIAS transform_types_fbs_cc)

# Target: hardware_module_state_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/hardware_module_state.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/hardware_module_state.fbs"
  COMMENT "Generating C++ Flatbuffers headers for hardware_module_state.fbs"
)
add_library(hardware_module_state_fbs_cc INTERFACE)
target_include_directories(hardware_module_state_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(hardware_module_state_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
)
target_link_libraries(hardware_module_state_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS hardware_module_state_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/hal/interfaces"
)
add_library(icon::hal::interfaces::hardware_module_state_fbs_cc ALIAS hardware_module_state_fbs_cc)

# Target: icon_state_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/icon_state.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/icon_state.fbs"
  COMMENT "Generating C++ Flatbuffers headers for icon_state.fbs"
)
add_library(icon_state_fbs_cc INTERFACE)
target_include_directories(icon_state_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(icon_state_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
)
target_link_libraries(icon_state_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS icon_state_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/hal/interfaces"
)
add_library(icon::hal::interfaces::icon_state_fbs_cc ALIAS icon_state_fbs_cc)

# Target: joint_command_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_command.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_command.fbs"
  COMMENT "Generating C++ Flatbuffers headers for joint_command.fbs"
)
add_library(joint_command_fbs_cc INTERFACE)
target_include_directories(joint_command_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(joint_command_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
)
target_link_libraries(joint_command_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS joint_command_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/hal/interfaces"
)
add_library(icon::hal::interfaces::joint_command_fbs_cc ALIAS joint_command_fbs_cc)

# Target: joint_limits_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_limits.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_limits.fbs"
  COMMENT "Generating C++ Flatbuffers headers for joint_limits.fbs"
)
add_library(joint_limits_fbs_cc INTERFACE)
target_include_directories(joint_limits_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(joint_limits_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
)
target_link_libraries(joint_limits_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS joint_limits_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/hal/interfaces"
)
add_library(icon::hal::interfaces::joint_limits_fbs_cc ALIAS joint_limits_fbs_cc)

# Target: joint_state_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_state.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/joint_state.fbs"
  COMMENT "Generating C++ Flatbuffers headers for joint_state.fbs"
)
add_library(joint_state_fbs_cc INTERFACE)
target_include_directories(joint_state_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(joint_state_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
)
target_link_libraries(joint_state_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS joint_state_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/hal/interfaces"
)
add_library(icon::hal::interfaces::joint_state_fbs_cc ALIAS joint_state_fbs_cc)

# Target: segment_info_fbs_cc
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
  COMMAND "${FLATC_EXECUTABLE}" --cpp --filename-suffix .fbs --keep-prefix --reflect-names --scoped-enums --gen-mutable --filename-ext h
          -o "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/interprocess/shared_memory_manager"
          -I "${CMAKE_CURRENT_LIST_DIR}/.."
          "${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/shared_memory_manager/segment_info.fbs"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/shared_memory_manager/segment_info.fbs"
  COMMENT "Generating C++ Flatbuffers headers for segment_info.fbs"
)
add_library(segment_info_fbs_cc INTERFACE)
target_include_directories(segment_info_fbs_cc INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
target_sources(segment_info_fbs_cc PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
)
target_link_libraries(segment_info_fbs_cc INTERFACE
  flatbuffers::flatbuffers
)
install(TARGETS segment_info_fbs_cc EXPORT icon_shared_memoryTargets)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
        DESTINATION "include/flatbuffer_definitions/icon/interprocess/shared_memory_manager"
)
add_library(icon::interprocess::shared_memory_manager::segment_info_fbs_cc ALIAS segment_info_fbs_cc)
