include(FetchContent)

# Fetch rules_cc for tools/cpp/runfiles/runfiles.h (matches MODULE.bazel: 0.2.22).
# TODO(wjwwood): Upstream Bazel deprecated @bazel_tools//tools/cpp/runfiles in favor of
# @rules_cc//cc/runfiles (#include "rules_cc/cc/runfiles/runfiles.h" in namespace
# rules_cc::cc::runfiles). In Bazel itself, tools/cpp/runfiles/runfiles.h is now a
# 10-line deprecated forwarding header that aliases rules_cc::cc::runfiles::Runfiles
# into bazel::tools::cpp::runfiles::Runfiles. Neither Bazel nor rules_cc ships a CMake
# or system package for cc/runfiles. If intrinsic/util/path_resolver.cc in the Intrinsic
# SDK is updated upstream to include "rules_cc/cc/runfiles/runfiles.h" (or if C++ targets
# are extracted from Bazel), the synthetic forwarding header below can be removed.
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
