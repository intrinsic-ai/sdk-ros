#pragma once

#include <type_traits>

#include "intrinsic/eigenmath/pose2.hpp"
#include "intrinsic/eigenmath/so2.hpp"
#include "intrinsic/eigenmath/so3.hpp"
#include "intrinsic/eigenmath/types.hpp"
#include "intrinsic/math/pose3.hpp"

namespace intrinsic {
namespace eigenmath {

// Checks if a type is Pose2<K>.
template <typename T>
static constexpr bool IsPose2 = false;

template <typename Scalar, int Options>
static constexpr bool IsPose2<Pose2<Scalar, Options>> = true;

// Checks if a type is Pose3<K>.
template <typename T>
static constexpr bool IsPose3 = false;

template <typename Scalar, int Options>
static constexpr bool IsPose3<Pose3<Scalar, Options>> = true;

// Checks if a type is Pose2<K> or Pose3<K>.
template <typename T>
static constexpr bool IsPose = IsPose2<T> || IsPose3<T>;

template <typename T>
struct ScalarTraitOf {
  // Use double if everything else fails.
  template <typename U, typename = void>
  struct ScalarTypeFromEigen {
    using type = double;
  };
  // If it's an Eigen type of some sort, there must be a norm since isApprox
  // depends on that function.
  template <typename U>
  struct ScalarTypeFromEigen<U,
                             std::void_t<decltype(std::declval<U>().norm())>> {
    using type = decltype(std::declval<U>().norm());
  };

  using type = typename ScalarTypeFromEigen<T>::type;
};

template <typename Scalar, int Options>
struct ScalarTraitOf<SO2<Scalar, Options>> {
  using type = Scalar;
};

template <typename Scalar, int Options>
struct ScalarTraitOf<SO3<Scalar, Options>> {
  using type = Scalar;
};

template <typename Scalar, int Options>
struct ScalarTraitOf<Pose2<Scalar, Options>> {
  using type = Scalar;
};

template <typename Scalar, int Options>
struct ScalarTraitOf<Pose3<Scalar, Options>> {
  using type = Scalar;
};

template <typename T>
using ScalarTypeOf = typename ScalarTraitOf<T>::type;

}  // namespace eigenmath
}  // namespace intrinsic
