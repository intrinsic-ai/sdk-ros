include(FetchContent)

# 1. Fetch riegeli (matches MODULE.bazel: 0.0.0-20250822-9f2744d)
FetchContent_Declare(
  riegeli
  URL https://github.com/google/riegeli/archive/9f2744d.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(riegeli)

# Compile riegeli's records_metadata.proto
set(RIEGELI_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/riegeli_proto_gen")
file(MAKE_DIRECTORY "${RIEGELI_PROTO_DIR}")
set(RIEGELI_PROTO_SRC "${riegeli_SOURCE_DIR}/riegeli/records/records_metadata.proto")
set(RIEGELI_PROTO_PB_CC "${RIEGELI_PROTO_DIR}/riegeli/records/records_metadata.pb.cc")
set(RIEGELI_PROTO_PB_H "${RIEGELI_PROTO_DIR}/riegeli/records/records_metadata.pb.h")

add_custom_command(
  OUTPUT "${RIEGELI_PROTO_PB_CC}" "${RIEGELI_PROTO_PB_H}"
  COMMAND protobuf::protoc
    --cpp_out=${RIEGELI_PROTO_DIR}
    -I ${riegeli_SOURCE_DIR}
    ${RIEGELI_PROTO_SRC}
  DEPENDS "${RIEGELI_PROTO_SRC}" protobuf::protoc
  COMMENT "Generating C++ protocol buffer for riegeli/records/records_metadata.proto"
  VERBATIM
)

file(GLOB_RECURSE riegeli_SRCS
  "${riegeli_SOURCE_DIR}/riegeli/base/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/brotli/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/bytes/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/bzip2/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/chunk_encoding/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/containers/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/digests/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/messages/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/ordered_varint/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/records/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/snappy/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/varint/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/zlib/*.cc"
  "${riegeli_SOURCE_DIR}/riegeli/zstd/*.cc"
)
list(FILTER riegeli_SRCS EXCLUDE REGEX "_test\\.cc$")
list(FILTER riegeli_SRCS EXCLUDE REGEX "_benchmark\\.cc$")
list(FILTER riegeli_SRCS EXCLUDE REGEX "/tools/")
list(APPEND riegeli_SRCS "${RIEGELI_PROTO_PB_CC}")

# 2. Fetch highwayhash (matches riegeli MODULE.bazel: 5ad3bf8)
FetchContent_Declare(
  highwayhash
  URL https://github.com/google/highwayhash/archive/5ad3bf8.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(highwayhash)

set(highwayhash_SRCS
  "${highwayhash_SOURCE_DIR}/highwayhash/arch_specific.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/c_bindings.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/hh_portable.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/instruction_sets.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/scalar_sip_tree_hash.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/sip_hash.cc"
  "${highwayhash_SOURCE_DIR}/highwayhash/sip_tree_hash.cc"
)
set_source_files_properties(
  "${highwayhash_SOURCE_DIR}/highwayhash/hh_portable.cc"
  PROPERTIES COMPILE_FLAGS "-DHH_TARGET_NAME=Portable"
)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64|amd64)|(^i.86$)")
  list(APPEND highwayhash_SRCS
    "${highwayhash_SOURCE_DIR}/highwayhash/hh_avx2.cc"
    "${highwayhash_SOURCE_DIR}/highwayhash/hh_sse41.cc"
  )
  set_source_files_properties(
    "${highwayhash_SOURCE_DIR}/highwayhash/hh_avx2.cc"
    "${highwayhash_SOURCE_DIR}/highwayhash/sip_tree_hash.cc"
    PROPERTIES COMPILE_FLAGS "-mavx2"
  )
  set_source_files_properties(
    "${highwayhash_SOURCE_DIR}/highwayhash/hh_sse41.cc"
    PROPERTIES COMPILE_FLAGS "-msse4.1"
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|arm)")
  list(APPEND highwayhash_SRCS
    "${highwayhash_SOURCE_DIR}/highwayhash/hh_neon.cc"
  )
endif()

# 3. Fetch tinygltf (matches MODULE.bazel: v2.9.6)
FetchContent_Declare(
  tinygltf
  URL https://github.com/syoyo/tinygltf/archive/refs/tags/v2.9.6.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(tinygltf)
set(tinygltf_SRCS "${tinygltf_SOURCE_DIR}/tiny_gltf.cc")

# 4. Fetch rules_cc for tools/cpp/runfiles/runfiles.h (matches MODULE.bazel: 0.2.22)
FetchContent_Declare(
  rules_cc
  URL https://github.com/bazelbuild/rules_cc/releases/download/0.2.22/rules_cc-0.2.22.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP FALSE
  SOURCE_SUBDIR non_existent_subdir
)
FetchContent_MakeAvailable(rules_cc)

set(RUNFILES_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/runfiles_include")
file(MAKE_DIRECTORY "${RUNFILES_INCLUDE_DIR}/rules_cc/cc/runfiles")
file(MAKE_DIRECTORY "${RUNFILES_INCLUDE_DIR}/tools/cpp/runfiles")
file(COPY "${rules_cc_SOURCE_DIR}/cc/runfiles/runfiles.h"
     DESTINATION "${RUNFILES_INCLUDE_DIR}/rules_cc/cc/runfiles")
file(WRITE "${RUNFILES_INCLUDE_DIR}/tools/cpp/runfiles/runfiles.h"
"#ifndef TOOLS_CPP_RUNFILES_RUNFILES_H_
#define TOOLS_CPP_RUNFILES_RUNFILES_H_
#include \"rules_cc/cc/runfiles/runfiles.h\"
namespace bazel {
namespace tools {
namespace cpp {
namespace runfiles {
using ::rules_cc::cc::runfiles::Runfiles;
}  // namespace runfiles
}  // namespace cpp
}  // namespace tools
}  // namespace bazel
#endif  // TOOLS_CPP_RUNFILES_RUNFILES_H_
")
set(rules_cc_runfiles_SRCS "${rules_cc_SOURCE_DIR}/cc/runfiles/runfiles.cc")
