#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "flatbuffers/array.h"
#include "flatbuffers/vector.h"

namespace intrinsic_fbs
{

// Helper function to get the number of elements of a flatbuffer struct's array
// member at compile time.
//
// This function takes a parameter `array_getter_ptr` of type
//                                             ╲
// const ArrayType* (FlatbufferStructT::*array_getter_ptr)() const
// └───────┬──────┘  └─────────┬────────┘                      │
//         │                   ├───────────────────────────────┘
//         │       pointer to const member function of FlatbufferStructT...
//         │
//  ...that returns const ArrayType*
//
// Note that the actual parameter doesn't have a name, because the function body
// does not use it, but for this explanation it's helpful to have _something_
// there.
//
// Usage:
//
// constexpr kMyArraySize =
//     FlatbufferArrayNumElements(
//         &::my::fbs::namespace::MyStruct::array_member);
//
// NOTE: You do *not* need to specify any of the template parameters, the
//       compiler can infer all three from the function pointer you pass into
//       this function.
template<typename FlatbufferStructT, typename ArrayT, uint16_t array_length>
constexpr size_t FlatbufferArrayNumElements(
  const flatbuffers::Array<ArrayT, array_length> * (FlatbufferStructT::*)() const)
{
  return array_length;
}

// Performs a deep copy of a flatbuffers::Vector.
//
// The template parameter `T` must be a simple flatbuffer type that can be
// copied trough std::copy.
// Returns OutOfRangeError if the `from` vector and the
// `to`vector have different sizes.
template<typename T>
intrinsic::hal::RealtimeStatus CopyFbsVector(
  const flatbuffers::Vector<T> & from, flatbuffers::Vector<T> & to)
{
  if (from.size() != to.size()) {
    intrinsic::hal::RealtimeStatus status;
    status.code = intrinsic::hal::StatusCode::kOutOfRange;
    (void) std::snprintf(
        status.message.data(),
        status.message.size(),
        "Vector sizes are not equal: %d != %d",
        from.size(),
        to.size());
    return status;
  }
  std::copy(from.data(), from.data() + from.size(), to.data());
  return {};
}

}  // namespace intrinsic_fbs
