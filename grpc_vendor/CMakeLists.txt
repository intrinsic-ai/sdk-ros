cmake_minimum_required(VERSION 3.10)
project(grpc_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)
find_package(absl REQUIRED)
find_package(Protobuf REQUIRED)
find_package(re2_vendor REQUIRED)
find_package(re2 REQUIRED)
find_package(bazelisk_vendor REQUIRED)

include(ExternalProject)

ament_vendor(grpc_vendor
  VCS_URL https://github.com/grpc/grpc
  VCS_VERSION v1.74.0
  PATCHES patches
  CMAKE_ARGS
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DABSL_BUILD_TESTING:BOOL=OFF
    -DABSL_ENABLE_INSTALL:BOOL=ON
    -DABSL_PROPAGATE_CXX_STD:BOOL=ON
    -DgRPC_INSTALL:BOOL=ON
    -DgRPC_ABSL_PROVIDER=package
    -DgRPC_RE2_PROVIDER=package
    -DgRPC_BUILD_TESTS:BOOL=OFF
    -DgRPC_BUILD_CSHARP_EXT:BOOL=OFF
    -DgRPC_BUILD_GRPC_CSHARP_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_GRPC_NODE_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_GRPC_PHP_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_GRPC_RUBY_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_NODE_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_OBJECTIVEC_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_RUBY_PLUGIN:BOOL=OFF
    -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=ON
    -DgRPC_PROTOBUF_PROVIDER:STRING=module
    -DPROTOBUF_ROOT_DIR=${PROTOBUF_ROOT_DIR}
  GLOBAL_HOOK
)

if (TARGET grpc_vendor)
  # Install the gRPC proto definitions so that we can use them elsewhere
  ExternalProject_Get_Property(grpc_vendor SOURCE_DIR INSTALL_DIR)

  install(DIRECTORY "${SOURCE_DIR}/src/proto/grpc" # source directory
          DESTINATION "opt/${PROJECT_NAME}/share/grpc-proto/src/proto/" # target directory
          FILES_MATCHING # install only matched files
          PATTERN "*.proto" # select header files
  )

  # Add a step to build and install the protobuf python package
  message(STATUS "bazelisk path: ${bazelisk_vendor_EXECUTABLE}")
  ExternalProject_Add_Step(grpc_vendor build_python_protobuf
    COMMAND ${bazelisk_vendor_EXECUTABLE} build --noenable_bzlmod //python/dist:binary_wheel
    DEPENDEES install
    WORKING_DIRECTORY ${SOURCE_DIR}/third_party/protobuf
  )
  ExternalProject_Add_Step(grpc_vendor install_python_protobuf
    COMMAND ${CMAKE_COMMAND} -DINSTALL_DIR=${INSTALL_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/install_protobuf_wheel.cmake
    DEPENDEES install
    WORKING_DIRECTORY ${SOURCE_DIR}/third_party/protobuf
)

  get_target_property(RE2_INCLUDE_DIRS re2::re2 INTERFACE_INCLUDE_DIRECTORIES)
  string(REPLACE ";" ":" RE2_INCLUDE_STR "${RE2_INCLUDE_DIRS}")
  message(STATUS "RE2 Include Dirs: ${RE2_INCLUDE_STR}")
  list(GET RE2_INCLUDE_DIRS 1 RE2_LIB_DIR)
  get_filename_component(RE2_LIB_DIR "${RE2_LIB_DIR}/../lib" ABSOLUTE)
  message(STATUS "RE2 Lib Dir: ${RE2_LIB_DIR}")

  get_target_property(ABSL_INCLUDE_DIRS absl::core_headers INTERFACE_INCLUDE_DIRECTORIES)
  string(REPLACE ";" ":" ABSL_INCLUDE_STR "${ABSL_INCLUDE_DIRS}")
  message(STATUS "ABSL Include Dirs: ${ABSL_INCLUDE_STR}")
  get_filename_component(ABSL_LIB_DIR "${ABSL_INCLUDE_DIRS}/../lib" ABSOLUTE)
  message(STATUS "ABSL Lib Dir: ${ABSL_LIB_DIR}")

  # Add a step to build and install the grpcio python package
  ExternalProject_Add_Step(grpc_vendor install_python_grpcio
    COMMAND ${CMAKE_COMMAND} -E env
            "GRPC_PYTHON_BUILD_SYSTEM_ABSL=TRUE"
            "GRPC_PYTHON_ABSL_INCLUDE=${ABSL_INCLUDE_STR}"
            "GRPC_PYTHON_ABSL_LIB_DIR=${ABSL_LIB_DIR}"
            "GRPC_PYTHON_BUILD_SYSTEM_RE2=TRUE"
            "GRPC_PYTHON_RE2_INCLUDE=${RE2_INCLUDE_STR}"
            "GRPC_PYTHON_RE2_LIB_DIR=${RE2_LIB_DIR}"
            "GRPC_PYTHON_BUILD_SYSTEM_ZLIB=TRUE"
            python3 -m pip install . --prefix=${INSTALL_DIR} --root=/ --no-deps --no-build-isolation
    DEPENDEES install_python_protobuf
    WORKING_DIRECTORY ${SOURCE_DIR}
  )

  ExternalProject_Add_Step(grpc_vendor install_python_grpcio_tools
    COMMAND ${SOURCE_DIR}/tools/distrib/python/make_grpcio_tools.py
    COMMAND ${CMAKE_COMMAND} -E env
            "GRPC_PYTHON_BUILD_WITH_CYTHON=TRUE"
            python3 -m pip install tools/distrib/python/grpcio_tools --prefix=${INSTALL_DIR} --root=/ --no-deps --no-build-isolation
    DEPENDEES install_python_grpcio
    WORKING_DIRECTORY ${SOURCE_DIR}
  )

  add_test(NAME test_grpc_import
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grpc_import.py
  )
  set_tests_properties(test_grpc_import PROPERTIES ENVIRONMENT
    "PYTHONPATH=${INSTALL_DIR}/local/lib/python3.12/dist-packages:$ENV{PYTHONPATH}"
  )
endif()

ament_environment_hooks(
  hooks/pythonpath.dsv.in
  hooks/pythonpath.sh.in
)

ament_export_dependencies(
  abseil_cpp_vendor
  re2_vendor
)

ament_package()
