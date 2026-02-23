#pragma once

#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <string_view>
#include <tl/expected.hpp>

#include "flatbuffers/array.h"
#include "intrinsic/utils/status.hpp"


namespace intrinsic_fbs
{

// Copies the contents of `source` to `destination`, up to the size of
// `destination` - 1 (to leave room for a trailing null byte). If `source` is
// shorter than `destination` - 1, the remaining bytes in `destination` are set
// to `\0`.
//
// The type `T` must be a character type, i.e. `char`, `unsigned char` or
// `signed char`.
// `size` is the size of the `destination` array and must be greater than 0.
// Returns an InvalidArgumentError if `destination` is a nullptr.
template<typename T, uint16_t size>
requires requires {
  std::is_same_v<T, char>|| std::is_same_v<T, unsigned char>||
  std::is_same_v<T, signed char>;
  size > 0;
}
intrinsic::RealtimeStatus StringCopy(
  ::flatbuffers::Array<T, size> * destination, std::string_view source)
{
  if (destination == nullptr) {
    return intrinsic::RealtimeStatus{
      .code = intrinsic::StatusCode::kInvalidArgument,
      .message = "destination must not be nullptr",
    };
  }
  // Zero-out the destination array just in case
  std::memset(destination->Data(), '\0', size);
  auto data = reinterpret_cast<char *>(destination->Data());
  (void) std::memcpy(
      data,
      source.data(),
      // We know from the requirements above that size is greater than zero, so
      // size-1 cannot be negative and it's safe to cast that to size_t.
      std::min(static_cast<size_t>(size - 1), source.size()));

  return intrinsic::RtOkStatus();
}


// Creates a string_view from the contents of `source`.
//
// The type `T` must be a character type, i.e. `char`, `unsigned char` or
// `signed char`.
// `size` is the size of the `source` array and must be greater than 0.
// The string_view extends to the first `\0` byte in `source`, or to the end
// of `source` if no `\0` byte is found, which might cause undefined behavior.
//
// Note that this function does not allocate memory, so the returned
// string_view is valid as long as the original buffer is still allocated.
//
// The function returns an InvalidArgumentError if `source` is a nullptr or if
// the source string does not end with a null byte.
template<typename T, uint16_t size>
requires requires {
  std::is_same_v<T, char>|| std::is_same_v<T, unsigned char>||
  std::is_same_v<T, signed char>;
  size > 0;
}
tl::expected<std::string_view, intrinsic::RealtimeStatus> StringView(
  const ::flatbuffers::Array<T, size> * source)
{
  if (source == nullptr) {
    return tl::unexpected(intrinsic::RealtimeStatus{
        .code = intrinsic::StatusCode::kInvalidArgument,
        .message = "source must not be nullptr",
      });
  }
  const T * data = source->data();
  size_t string_size = size;
  for (size_t i = 0; i < size; ++i) {
    if (data[i] == '\0') {
      string_size = i;
      break;
    }
  }
  return std::string_view{reinterpret_cast<const char *>(source->Data()),
    string_size};
}


// Helper traits to get the size of a flatbuffers array.
namespace internal
{
namespace flatbuffers
{
template<typename>
struct ArraySize;

template<typename T, uint16_t size_value>
struct ArraySize<::flatbuffers::Array<T, size_value>>
{
  static constexpr uint16_t value = size_value;
};
}  // namespace flatbuffers
}  // namespace internal

// Enable the function if T is a flatbuffers array, or more specifically,
// - if the array has a 1st field named "data" that is a pointer to a int8_t or
//   uint8_t array.
// - if the array has another field named "size" that is unsigned.
template<typename T,
  typename ArrayType = std::remove_cv_t<
    std::remove_pointer_t<typename T::Traits::template FieldType<0>>>,
  uint16_t MaxSize = internal::flatbuffers::ArraySize<ArrayType>::value>
requires requires(T & obj)
{
  requires std::is_convertible_v<decltype(obj.mutable_data()),
    ::flatbuffers::Array<int8_t, MaxSize> *>||
  std::is_convertible_v<decltype(obj.mutable_data()),
    ::flatbuffers::Array<uint8_t, MaxSize> *>;
  requires std::is_unsigned_v<decltype(obj.size())>;
  {
    obj.mutate_size(std::declval<decltype(obj.size())>())
  }->std::same_as<void>;
}
void StringCopy(T * destination, std::string_view source)
{
  if (destination == nullptr) {
    return;
  }
  std::memcpy(destination->mutable_data()->Data(),
              source.data(),
              source.size());
  destination->mutate_size(source.size());
}

// Enable the function if T is a flatbuffers array, or more specifically,
// - if the array has a 1st field named "data" that is a pointer to a int8_t or
//   uint8_t array.
// - if the array has another field named "size" that is unsigned.
template<typename T,
  typename ArrayType = std::remove_cv_t<
    std::remove_pointer_t<typename T::Traits::template FieldType<0>>>,
  uint16_t MaxSize = internal::flatbuffers::ArraySize<ArrayType>::value>
requires requires(T & obj)
{
  requires std::is_convertible_v<
    decltype(obj.data()),
    const ::flatbuffers::Array<int8_t, MaxSize> *>||
  std::is_convertible_v<
    decltype(obj.data()),
    const ::flatbuffers::Array<uint8_t, MaxSize> *>;
  requires std::is_unsigned_v<decltype(obj.size())>;
}

std::string_view StringView(const T * source)
{
  if (source == nullptr) {
    return std::string_view();
  }
  return std::string_view(
      reinterpret_cast<const char *>(source->data()->data()), source->size());
}

}  // namespace intrinsic_fbs
