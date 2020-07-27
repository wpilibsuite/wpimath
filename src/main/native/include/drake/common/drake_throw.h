#pragma once

#include <type_traits>

#include "drake/common/drake_assert.h"

#ifndef WPIMATHEXPORT
#ifdef BUILDING_WPIMATH
#include "wpimath_exports.h"
#else
#define WPIMATHEXPORT
#endif
#endif

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// ::abort(), and cannot be disabled.

namespace drake {
namespace internal {
// Throw an error message.
[[noreturn]]
WPIMATHEXPORT void Throw(const char* condition, const char* func, const char* file, int line);
}  // namespace internal
}  // namespace drake

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
#define DRAKE_THROW_UNLESS(condition)                                        \
  do {                                                                       \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv<decltype(condition)>::type> Trait;           \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    if (!Trait::Evaluate(condition)) {                                       \
      ::drake::internal::Throw(#condition, __func__, __FILE__, __LINE__);    \
    }                                                                        \
  } while (0)
