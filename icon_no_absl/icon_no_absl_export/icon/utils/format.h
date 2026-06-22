#ifndef ICON_UTILS_FORMAT_H_
#define ICON_UTILS_FORMAT_H_

#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

namespace intrinsic {

namespace traits {
template <class T>
concept Streamable = requires(std::ostream os, T a) {
  { os << a };
};
}  // namespace traits

// Formats an unordered_map as a string.
//
// Example:
// auto s = FormatMap(std::unordered_map<int, string>{
//     {1, "one"},
//     {2, "duo"},
//     {3, "三"},
// };
//
// -> "1=one, 2=duo, 3=三"
//
// Both K and V must have std::ostream::operator<<.
//
// The remaining parameters are optional. Use them to tweak the output and make
// it pretty.
template <traits::Streamable K, traits::Streamable V>
std::string FormatMap(const std::unordered_map<K, V>& map,
                      std::string_view element_separator = ", ",
                      std::string_view pair_separator = "=",
                      std::string_view key_quotes = "",
                      std::string_view value_quotes = "") {
  std::stringstream s;
  bool first = true;
  for (const auto& [k, v] : map) {
    if (!first) {
      s << element_separator;
    } else {
      first = false;
    }
    s << key_quotes << k << key_quotes << pair_separator << value_quotes << v
      << value_quotes;
  }
  return s.str();
}

}  // namespace intrinsic

#endif  // ICON_UTILS_FORMAT_H_
