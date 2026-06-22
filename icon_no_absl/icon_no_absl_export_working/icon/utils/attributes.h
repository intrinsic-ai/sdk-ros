#ifndef ICON_UTILS_ATTRIBUTES_H_
#define ICON_UTILS_ATTRIBUTES_H_

#ifdef __has_attribute
#define INTR_HAS_ATTRIBUTE(x) __has_attribute(x)
#else
#define INTR_HAS_ATTRIBUTE(x) 0
#endif

#ifdef __has_cpp_attribute
#define INTR_HAS_CPP_ATTRIBUTE(x) __has_cpp_attribute(x)
#else
#define INTR_HAS_CPP_ATTRIBUTE(x) 0
#endif

#if INTR_HAS_CPP_ATTRIBUTE(nodiscard)
#define INTR_MUST_USE_RESULT [[nodiscard]]
#elif INTR_HAS_ATTRIBUTE(warn_unused_result)
#define INTR_MUST_USE_RESULT __attribute__((warn_unused_result))
#else
#define INTR_MUST_USE_RESULT
#endif

#if INTR_HAS_CPP_ATTRIBUTE(clang::lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND [[clang::lifetimebound]]
#elif INTR_HAS_CPP_ATTRIBUTE(msvc::lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND [[msvc::lifetimebound]]
#elif INTR_HAS_ATTRIBUTE(lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND __attribute__((lifetimebound))
#else
#define INTR_ATTRIBUTE_LIFETIME_BOUND
#endif

#if INTR_HAS_CPP_ATTRIBUTE(clang::lifetime_capture_by)
#define INTR_ATTRIBUTE_LIFETIME_CAPTURE_BY(x) [[clang::lifetime_capture_by(x)]]
#else
#define INTR_ATTRIBUTE_LIFETIME_CAPTURE_BY(x)
#endif

#endif  // ICON_UTILS_ATTRIBUTES_H_
