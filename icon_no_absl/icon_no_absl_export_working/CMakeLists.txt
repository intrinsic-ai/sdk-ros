cmake_minimum_required(VERSION 3.19)
project(icon_shared_memory CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 1. Setup packages and tools
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(tl-expected REQUIRED)
find_program(FLATC_EXECUTABLE flatc REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/flatbuffer_definitions/targets.cmake")

enable_testing()
include(GoogleTest)

# Define repository root
get_filename_component(INSRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

# 2. Include targets in topological dependency order
include("${CMAKE_CURRENT_LIST_DIR}/eigenmath/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/utils/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/control/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/flatbuffers/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/shared_memory_manager/testing/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/util/thread/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/shared_memory_manager/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/kinematics/types/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/hal/interfaces/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/shared_memory_lockstep/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/hal/targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/icon/interprocess/remote_trigger/targets.cmake")

# 3. Generate & Install CMake Package Files
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfig.cmake"
  "include(CMakeFindDependencyMacro)\n"
  "find_dependency(Eigen3)\n"
  "find_dependency(GTest)\n"
  "find_dependency(flatbuffers)\n"
  "find_dependency(tl-expected)\n"
  "include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/icon_shared_memoryTargets.cmake\")\n"
)

install(EXPORT icon_shared_memoryTargets
        FILE icon_shared_memoryTargets.cmake
        DESTINATION lib/cmake/icon_shared_memory
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/icon_shared_memoryConfigVersion.cmake"
  DESTINATION lib/cmake/icon_shared_memory
)
