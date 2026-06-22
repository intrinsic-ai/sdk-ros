#ifndef ICON_UTILS_STATUS_AND_EXPECTED_TEST_MACROS_H_
#define ICON_UTILS_STATUS_AND_EXPECTED_TEST_MACROS_H_

#include <tl/expected.hpp>

#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

// This file contains macros to help work with `intrinsic::Status`,
// `intrinsic::RealtimeStatus` and `tl::expected` values whose `Error` is one of
// the two status structs.
//
// The macros are modelled on Abseil's test macros
// (https://github.com/abseil/abseil-cpp/blob/d1482738eef0a06be9251314a3263db550be33c7/absl/status/status_matchers.h).

#define INTR_ASSERT_OK(expr)                                                 \
  do {                                                                       \
    auto converter = ::intrinsic::icon::status_macro_internal::ToBool(expr); \
    ASSERT_TRUE(converter.ok())                                              \
        << ::intrinsic::ToString(converter.GetStatus());                     \
  } while (0)

#define INTR_EXPECT_OK(expr)                                                 \
  do {                                                                       \
    auto converter = ::intrinsic::icon::status_macro_internal::ToBool(expr); \
    EXPECT_TRUE(converter.ok())                                              \
        << ::intrinsic::ToString(converter.GetStatus());                     \
  } while (0)

#define INTR_ASSERT_OK_AND_ASSIGN(lhs, expr)                               \
  INTR_INTERNAL_ASSERT_OK_AND_ASSIGN_IMPL(                                 \
      INTR_INTERNAL_STATUS_MACROS_CONCAT(_expected_value_, __LINE__), lhs, \
      expr)

#define INTR_INTERNAL_ASSERT_OK_AND_ASSIGN_IMPL(expected_varname, lhs, expr) \
  auto expected_varname = (expr);                                            \
  ASSERT_TRUE(expected_varname.has_value())                                  \
      << ::intrinsic::ToString(expected_varname.error());                    \
  INTR_INTERNAL_STATUS_MACROS_IMPL_UNPARENTHESIZE_IF_PARENTHESIZED_(lhs) =   \
      std::move(expected_varname.value())

#endif  // ICON_UTILS_STATUS_AND_EXPECTED_TEST_MACROS_H_
