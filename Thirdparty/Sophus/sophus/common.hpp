/// @file
/// Common functionality.

#ifndef SOPHUS_COMMON_HPP
#define SOPHUS_COMMON_HPP

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <type_traits>

#include <Eigen/Core>

#if !defined(SOPHUS_DISABLE_ENSURES)
#include "formatstring.hpp"
#endif //!defined(SOPHUS_DISABLE_ENSURES)

// following boost's assert.hpp
#undef SOPHUS_ENSURE

// ENSURES are similar to ASSERTS, but they are always checked for (including in
// release builds). At the moment there are no ASSERTS in Sophus which should
// only be used for checks which are performance critical.

#ifdef __GNUC__
#define SOPHUS_FUNCTION __PRETTY_FUNCTION__
#elif (_MSC_VER >= 1310)
#define SOPHUS_FUNCTION __FUNCTION__
#else
#define SOPHUS_FUNCTION "unknown"
#endif

// Make sure this compiles with older versions of Eigen which do not have
// EIGEN_DEVICE_FUNC defined.
#ifndef EIGEN_DEVICE_FUNC
#define EIGEN_DEVICE_FUNC
#endif

// NVCC on windows has issues with defaulting the Map specialization constructors, so
// special case that specific platform case.
#if defined(_MSC_VER) && defined(__CUDACC__)
#define SOPHUS_WINDOW_NVCC_FALLBACK
#endif

#define SOPHUS_FUNC EIGEN_DEVICE_FUNC

#if defined(SOPHUS_DISABLE_ENSURES)

#define SOPHUS_ENSURE(expr, ...) ((void)0)

#elif defined(SOPHUS_ENABLE_ENSURE_HANDLER)

namespace Sophus {
void ensureFailed(char const* function, char const* file, int line,
                  char const* description);
}

#define SOPHUS_ENSURE(expr, ...)                     \
  ((expr) ? ((void)0)                                \
          : ::Sophus::ensureFailed(                  \
                SOPHUS_FUNCTION, __FILE__, __LINE__, \
                Sophus::details::FormatString(__VA_ARGS__).c_str()))
#else
// LCOV_EXCL_START

namespace Sophus {
template <class... Args>
SOPHUS_FUNC void defaultEnsure(char const* function, char const* file, int line,
                               char const* description, Args&&... args) {
  std::printf("Sophus ensure failed in function '%s', file '%s', line %d.\n",
              function, file, line);
#ifdef __CUDACC__
  std::printf("%s", description);
#else
  std::cout << details::FormatString(description, std::forward<Args>(args)...)
            << std::endl;
  std::abort();
#endif
}
}  // namespace Sophus

// LCOV_EXCL_STOP
#define SOPHUS_ENSURE(expr, ...)                                       \
  ((expr) ? ((void)0)                                                  \
          : Sophus::defaultEnsure(SOPHUS_FUNCTION, __FILE__, __LINE__, \
                                  ##__VA_ARGS__))
#endif

namespace Sophus {

template <class Scalar>
struct Constants {
  SOPHUS_FUNC static Scalar epsilon() { return Scalar(1e-10); }

  SOPHUS_FUNC static Scalar epsilonSqrt() {
    using std::sqrt;
    return sqrt(epsilon());
  }

  SOPHUS_FUNC static Scalar pi() {
    return Scalar(3.141592653589793238462643383279502884);
  }
};

template <>
struct Constants<float> {
  SOPHUS_FUNC static float constexpr epsilon() {
    return static_cast<float>(1e-5);
  }

  SOPHUS_FUNC static float epsilonSqrt() { return std::sqrt(epsilon()); }

  SOPHUS_FUNC static float constexpr pi() {
    return 3.141592653589793238462643383279502884f;
  }
};

/// Nullopt type of lightweight optional class.
struct nullopt_t {
  explicit constexpr nullopt_t() {}
};

constexpr nullopt_t nullopt{};

/// Lightweight optional implementation which requires ``T`` to have a
/// default constructor.
///
/// TODO: Replace with std::optional once Sophus moves to c++17.
///
template <class T>
class optional {
 public:
  optional() : is_valid_(false) {}

  optional(nullopt_t) : is_valid_(false) {}

  optional(T const& type) : type_(type), is_valid_(true) {}

  explicit operator bool() const { return is_valid_; }

  T const* operator->() const {
    SOPHUS_ENSURE(is_valid_, "must be valid");
    return &type_;
  }

  T* operator->() {
    SOPHUS_ENSURE(is_valid_, "must be valid");
    return &type_;
  }

  T const& operator*() const {
    SOPHUS_ENSURE(is_valid_, "must be valid");
    return type_;
  }

  T& operator*() {
    SOPHUS_ENSURE(is_valid_, "must be valid");
    return type_;
  }

 private:
  T type_;
  bool is_valid_;
};

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <class G>
struct IsUniformRandomBitGenerator {
  static const bool value = std::is_unsigned<typename G::result_type>::value &&
                            std::is_unsigned<decltype(G::min())>::value &&
                            std::is_unsigned<decltype(G::max())>::value;
};
}  // namespace Sophus

#endif  // SOPHUS_COMMON_HPP
