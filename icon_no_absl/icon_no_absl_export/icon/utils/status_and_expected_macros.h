#ifndef ICON_UTILS_STATUS_AND_EXPECTED_MACROS_H_
#define ICON_UTILS_STATUS_AND_EXPECTED_MACROS_H_

// We use `expected` to provide functionality similar to `absl::StatusOr`.
// Since `std::expected  is only available in C++23, we rely on `tl::expected`
// as long as we need to support older C++ standards (like those used by ROS2)
#include <tl/expected.hpp>
#include <utility>

#include "icon/utils/status.h"

// This file contains macros to help work with `intrinsic::Status`,
// `intrinsic::RealtimeStatus` and `tl::expected` values whose `Error` is one of
// the two status structs.
//
// The macros are modelled on Abseil's status macros
// (https://github.com/abseil/abseil-cpp/blob/d1482738eef0a06be9251314a3263db550be33c7/absl/status/status_macros.h).

// All of these helper structs have a similar interface:
// * `ok()` returns a bool
// * `GetUnexpected()` returns either `tl::unexpected<Status>` or
// `tl::unexpected<RealtimeStatus>`
// * `GetStatus()` returns either `Status` or `RealtimeStatus`
//
// Because of the similar interfaces (and the similarity between `Status` and
// `RealtimeStatus`), the macros below can work with both `Status` and
// `RealtimeStatus`, so users only need to remember one macro name.

namespace intrinsic::icon::status_macro_internal {
class StatusToBool {
 public:
  constexpr explicit StatusToBool(Status s) : status_(std::move(s)) {}

  constexpr bool ok() const { return status_.ok(); }
  constexpr tl::unexpected<Status> GetUnexpected() {
    return tl::unexpected(status_);
  }
  constexpr const Status& GetStatus() { return status_; }

 private:
  Status status_;
};

class RealtimeStatusToBool {
 public:
  constexpr explicit RealtimeStatusToBool(RealtimeStatus s)
      : status_(std::move(s)) {}

  constexpr bool ok() const { return status_.ok(); }
  constexpr tl::unexpected<RealtimeStatus> GetUnexpected() {
    return tl::unexpected(status_);
  }
  constexpr const RealtimeStatus& GetStatus() { return status_; }

 private:
  RealtimeStatus status_;
};

template <class T>
class ExpectedStatusToBool {
 public:
  using expected_t = tl::expected<T, Status>;
  constexpr explicit ExpectedStatusToBool(expected_t e)
      : expected_(std::move(e)) {}

  constexpr bool ok() const { return expected_.has_value(); }
  constexpr tl::unexpected<Status> GetUnexpected() {
    return tl::unexpected(expected_.error());
  }
  constexpr const Status& GetStatus() { return expected_.error(); }

 private:
  expected_t expected_;
};

template <class T>
class ExpectedRealtimeStatusToBool {
 public:
  using expected_t = tl::expected<T, RealtimeStatus>;
  constexpr explicit ExpectedRealtimeStatusToBool(expected_t e)
      : expected_(std::move(e)) {}

  constexpr bool ok() const { return expected_.has_value(); }
  constexpr tl::unexpected<RealtimeStatus> GetUnexpected() {
    return tl::unexpected(expected_.error());
  }
  constexpr const RealtimeStatus& GetStatus() { return expected_.error(); }

 private:
  expected_t expected_;
};

// Overloaded helper function so that the macros below work with
// * Status
// * RealtimeStatus
// * tl::expected<T, Status>
// * tl::expected<T, RealtimeStatus>

inline constexpr StatusToBool ToBool(Status s) {
  return StatusToBool(std::move(s));
}
inline constexpr RealtimeStatusToBool ToBool(RealtimeStatus s) {
  return RealtimeStatusToBool(std::move(s));
}
template <class T>
inline constexpr ExpectedStatusToBool<T> ToBool(tl::expected<T, Status> e) {
  return ExpectedStatusToBool<T>(std::move(e));
}
template <class T>
inline constexpr ExpectedRealtimeStatusToBool<T> ToBool(
    tl::expected<T, RealtimeStatus> e) {
  return ExpectedRealtimeStatusToBool<T>(std::move(e));
}

}  // namespace intrinsic::icon::status_macro_internal

// Use this to quickly define an early return from a function that returns
// `Status` or `RealtimeStatus`.
//
// Doing so can reduce the complexity of functions and make them easier to read!
//
// For example:
// ```c++
// intrinsic::Status DoThing(int x) {
//   INTR_RETURN_STATUS_IF_ERROR(PrepareThing(x));
//   // We know that `PrepareThing()` succeeded, carry on.
//   return ActuallyDoThing(x);
// }
// ```
#define INTR_RETURN_STATUS_IF_ERROR(expr)                           \
  do {                                                              \
    if (auto converter =                                            \
            ::intrinsic::icon::status_macro_internal::ToBool(expr); \
        !converter.ok()) {                                          \
      return converter.GetStatus();                                 \
    }                                                               \
  } while (0)

// Use this to quickly define an early return from a function that returns
// `tl::expected<T, Status>` or `tl::expected<T, RealtimeStatus>`
#define INTR_RETURN_UNEXPECTED_IF_ERROR(expr)                       \
  do {                                                              \
    if (auto converter =                                            \
            ::intrinsic::icon::status_macro_internal::ToBool(expr); \
        !converter.ok()) {                                          \
      return converter.GetUnexpected();                             \
    }                                                               \
  } while (0)

// Use this to quickly check the return value of a function that returns
// `tl::expected<T, Status>` or `tl::expected<T, RealtimeStatus>`, and propagate
// the status in case of an error.
//
// For example:
// ```c++
// intrinsic::Status DoThing(int x) {
//   INTR_ASSIGN_OR_RETURN_STATUS(auto prepared, PrepareThing(x));
//   // We know that `PrepareThing()` succeeded, so we can use `prepared`.
//   return ActuallyDoThing(prepared);
// }
// ```
#define INTR_ASSIGN_OR_RETURN_STATUS(lhs, expr)                            \
  INTR_INTERNAL_ASSIGN_OR_RETURN_STATUS_IMPL(                              \
      INTR_INTERNAL_STATUS_MACROS_CONCAT(_expected_value_, __LINE__), lhs, \
      expr)

#define INTR_INTERNAL_ASSIGN_OR_RETURN_STATUS_IMPL(expected_varname, lhs,  \
                                                   expr)                   \
  auto expected_varname = (expr);                                          \
  if (!expected_varname.has_value()) {                                     \
    return expected_varname.error();                                       \
  }                                                                        \
  INTR_INTERNAL_STATUS_MACROS_IMPL_UNPARENTHESIZE_IF_PARENTHESIZED_(lhs) = \
      std::move(expected_varname.value())

// Use this to quickly check the return value of a function that returns
// `tl::expected<T, Status>` or `tl::expected<T, RealtimeStatus>`, and propagate
// the status as a `tl::unexpected` value in case of an error.
//
// For example:
// ```c++
// tl::expected<int, intrinsic::Status> DoThing(int x) {
//   INTR_ASSIGN_OR_RETURN_UNEXPECTED(auto prepared, PrepareThing(x));
//   // We know that `PrepareThing()` succeeded, so we can use `prepared`.
//   return ActuallyDoThing(prepared);
// }
// ```
#define INTR_ASSIGN_OR_RETURN_UNEXPECTED(lhs, expr)                        \
  INTR_INTERNAL_ASSIGN_OR_RETURN_UNEXPECTED_IMPL(                          \
      INTR_INTERNAL_STATUS_MACROS_CONCAT(_expected_value_, __LINE__), lhs, \
      expr)

#define INTR_INTERNAL_ASSIGN_OR_RETURN_UNEXPECTED_IMPL(expected_varname, lhs, \
                                                       expr)                  \
  auto expected_varname = (expr);                                             \
  if (!expected_varname.has_value()) {                                        \
    return tl::unexpected(expected_varname.error());                          \
  }                                                                           \
  INTR_INTERNAL_STATUS_MACROS_IMPL_UNPARENTHESIZE_IF_PARENTHESIZED_(lhs) =    \
      std::move(expected_varname.value())

// Internal helpers for macro expansion.
#define INTR_INTERNAL_STATUS_MACROS_IMPL_EAT_(...)
#define INTR_INTERNAL_STATUS_MACROS_IMPL_REM_(...) __VA_ARGS__
#define INTR_INTERNAL_STATUS_MACROS_IMPL_EMPTY_()

// Internal helpers for emptyness arguments check. Expands to "0_" if empty and
// otherwise "1_".
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_(...) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_HELPER_((__VA_ARGS__, 0_, 1_))
// MSVC expands variadic macros incorrectly, so we need this extra indirection
// to work around that.
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_HELPER_(args) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_I_ args
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_I_(e0, e1, is_empty, \
                                                           ...)              \
  is_empty

#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_(...) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_I_(__VA_ARGS__)
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_I_(...) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_INNER_(_, ##__VA_ARGS__)

// Internal helpers for if statement.
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IF_1_(_Then, _Else) _Then
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IF_0_(_Then, _Else) _Else
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IF_(_Cond, _Then, _Else)          \
  INTR_INTERNAL_STATUS_MACROS_CONCAT(INTR_INTERNAL_STATUS_MACROS_IMPL_IF_, \
                                     _Cond)(_Then, _Else)

// Expands to 1_ if the input is parenthesized. Otherwise expands to 0_.
#define INTR_INTERNAL_STATUS_MACROS_IMPL_IS_PARENTHESIZED_(...) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IS_EMPTY_(                   \
      INTR_INTERNAL_STATUS_MACROS_IMPL_EAT_ __VA_ARGS__)

// If the input is parenthesized, removes the parentheses. Otherwise expands to
// the input unchanged.
#define INTR_INTERNAL_STATUS_MACROS_IMPL_UNPARENTHESIZE_IF_PARENTHESIZED_(...) \
  INTR_INTERNAL_STATUS_MACROS_IMPL_IF_(                                        \
      INTR_INTERNAL_STATUS_MACROS_IMPL_IS_PARENTHESIZED_(__VA_ARGS__),         \
      INTR_INTERNAL_STATUS_MACROS_IMPL_REM_,                                   \
      INTR_INTERNAL_STATUS_MACROS_IMPL_EMPTY_())                               \
  __VA_ARGS__

#define INTR_INTERNAL_STATUS_MACROS_CONCAT_INNER(x, y) x##y
#define INTR_INTERNAL_STATUS_MACROS_CONCAT(x, y) \
  INTR_INTERNAL_STATUS_MACROS_CONCAT_INNER(x, y)

#endif  // ICON_UTILS_STATUS_AND_EXPECTED_MACROS_H_
