include(FetchContent)
FetchContent_Declare(
  googleapis
  GIT_REPOSITORY https://github.com/googleapis/googleapis
  GIT_TAG        master
)
FetchContent_MakeAvailable(googleapis)

FetchContent_Declare(
  grpc_gateway
  GIT_REPOSITORY https://github.com/grpc-ecosystem/grpc-gateway
  GIT_TAG        main
)
FetchContent_MakeAvailable(grpc_gateway)

FetchContent_Declare(
  cel_spec
  GIT_REPOSITORY https://github.com/google/cel-spec
  GIT_TAG        v0.25.1
)
FetchContent_MakeAvailable(cel_spec)
