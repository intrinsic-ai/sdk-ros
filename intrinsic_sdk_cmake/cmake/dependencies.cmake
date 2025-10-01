# find_package() all the dependencies, this is used by the intrinsic_sdk_cmakeConfig.cmake too.

# Find vendor packages to ensure dependencies are available.
find_package(abseil_cpp_vendor REQUIRED)
find_package(bazelisk_vendor REQUIRED)
find_package(eigen_vendor REQUIRED)
find_package(flatbuffers_vendor REQUIRED)
find_package(grpc_vendor REQUIRED)  # Provides protobuf and abseilcpp
find_package(gz_cmake_vendor REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_utils_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(intrinsic_pybind11_vendor REQUIRED)
find_package(pybind11_abseil_vendor REQUIRED)
find_package(pybind11_protobuf_vendor REQUIRED)
find_package(ortools_vendor REQUIRED)
find_package(opencensus_cpp_vendor REQUIRED)

# Find project dependencies.
find_package(Eigen3 REQUIRED)
find_package(flatbuffers REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz-transport14 REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python COMPONENTS Development)
find_package(pybind11_abseil CONFIG REQUIRED)
find_package(pybind11_protobuf CONFIG REQUIRED)
find_package(ortools CONFIG REQUIRED)
find_package(OpenCensus REQUIRED)
