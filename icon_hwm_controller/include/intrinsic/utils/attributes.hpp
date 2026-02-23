#pragma once

#ifdef __has_attribute
#define INTR_HAVE_ATTRIBUTE(x) __has_attribute(x)
#else
#define INTR_HAVE_ATTRIBUTE(x) 0
#endif

#ifdef __has_cpp_attribute
#define INTR_HAVE_CPP_ATTRIBUTE(x) __has_cpp_attribute(x)
#else
#define INTR_HAVE_CPP_ATTRIBUTE(x) 0
#endif

#if INTR_HAVE_CPP_ATTRIBUTE(nodiscard)
#define INTR_MUST_USE_RESULT [[nodiscard]]
#else
#define INTR_MUST_USE_RESULT
#endif

#if INTR_HAVE_CPP_ATTRIBUTE(clang::lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND [[clang::lifetimebound]]
#elif INTR_HAVE_CPP_ATTRIBUTE(msvc::lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND [[msvc::lifetimebound]]
#elif INTR_HAVE_ATTRIBUTE(lifetimebound)
#define INTR_ATTRIBUTE_LIFETIME_BOUND __attribute__((lifetimebound))
#else
#define INTR_ATTRIBUTE_LIFETIME_BOUND
#endif
