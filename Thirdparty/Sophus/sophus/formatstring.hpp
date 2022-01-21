/// @file
/// FormatString functionality

#ifndef SOPHUS_FORMATSTRING_HPP
#define SOPHUS_FORMATSTRING_HPP

#include <iostream>

namespace Sophus {
namespace details {

// Following: http://stackoverflow.com/a/22759544
template <class T>
class IsStreamable {
 private:
  template <class TT>
  static auto test(int)
      -> decltype(std::declval<std::stringstream&>() << std::declval<TT>(),
                  std::true_type());

  template <class>
  static auto test(...) -> std::false_type;

 public:
  static bool const value = decltype(test<T>(0))::value;
};

template <class T>
class ArgToStream {
 public:
  static void impl(std::stringstream& stream, T&& arg) {
    stream << std::forward<T>(arg);
  }
};

inline void FormatStream(std::stringstream& stream, char const* text) {
  stream << text;
  return;
}

// Following: http://en.cppreference.com/w/cpp/language/parameter_pack
template <class T, typename... Args>
void FormatStream(std::stringstream& stream, char const* text, T&& arg,
                  Args&&... args) {
  static_assert(IsStreamable<T>::value,
                "One of the args has no ostream overload!");
  for (; *text != '\0'; ++text) {
    if (*text == '%') {
      ArgToStream<T&&>::impl(stream, std::forward<T>(arg));
      FormatStream(stream, text + 1, std::forward<Args>(args)...);
      return;
    }
    stream << *text;
  }
  stream << "\nFormat-Warning: There are " << sizeof...(Args) + 1
         << " args unused.";
  return;
}

template <class... Args>
std::string FormatString(char const* text, Args&&... args) {
  std::stringstream stream;
  FormatStream(stream, text, std::forward<Args>(args)...);
  return stream.str();
}

inline std::string FormatString() { return std::string(); }

}  // namespace details
}  // namespace Sophus

#endif //SOPHUS_FORMATSTRING_HPP
