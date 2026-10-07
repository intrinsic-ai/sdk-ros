include(FetchContent)
FetchContent_Declare(
  googleapis
  URL https://github.com/googleapis/googleapis/archive/refs/heads/master.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(googleapis)

FetchContent_Declare(
  grpc_gateway
  URL https://github.com/grpc-ecosystem/grpc-gateway/archive/refs/heads/main.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(grpc_gateway)

FetchContent_Declare(
  cel_spec
  URL https://github.com/google/cel-spec/archive/refs/tags/v0.25.1.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(cel_spec)
