/// @brief Minimal formatting utility to replace boost::format using C++17 streams.
///
/// Features (subset of printf):
///  - Integers: %d, %i, %u with optional zero-padding width (e.g. %04d)
///  - Floating: %f with optional precision (e.g. %.3f)
///  - Strings:  %s
///  - Literal percent: %%
///
/// Syntax: %0[width][.precision][specifier]
/// Example: format("%04d-%.2f", 7, 3.14159) -> "0007-3.14"

#pragma once

#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace utility {

/// @brief Internal helpers used by format(). Most users should call utility::format().
namespace detail {

/// @brief Append literal segment from the format string.
/// @param out Destination string to append to.
/// @param fmt The full format string.
/// @param start Inclusive start index in fmt.
/// @param end Exclusive end index in fmt.
void append_literal(std::string& out, const std::string& fmt, std::size_t start, std::size_t end);

/// @brief Stream a generic value honoring width/precision for floating specs.
/// @tparam T Any value streamable to std::ostringstream.
/// @param oss Destination stream.
/// @param spec Format specifier (e.g. 'f', 'd', 's').
/// @param width Minimum field width (or -1 for none).
/// @param zero_pad Enable zero padding when width > 0.
/// @param precision Precision for floating types (or -1 for none).
/// @param value Value to stream.
template <typename T>
inline void stream_value(std::ostringstream& oss, char spec, int width, bool zero_pad, int precision, const T& value) {
  if (width > 0) {
    oss << std::setw(width) << (zero_pad ? std::setfill('0') : std::setfill(' '));
  }
  if (precision >= 0 && (spec == 'f' || spec == 'g' || spec == 'e' || spec == 'E')) {
    oss << std::fixed << std::setprecision(precision);
  }
  oss << value;
}

/// @brief Overload for C-strings.
/// @param oss Destination stream.
/// @param spec Format specifier.
/// @param width Minimum field width (or -1 for none).
/// @param zero_pad Enable zero padding when width > 0.
/// @param precision Precision for floating types (or -1 for none).
/// @param value Null-terminated string to stream.
void stream_value(std::ostringstream& oss, char spec, int width, bool zero_pad, int precision, const char* value);

/// @brief Parse a single format spec starting at position pos (after '%').
/// @param fmt The format string.
/// @param pos Index right after the leading '%'.
/// @param zero_pad Output: whether zero padding is requested.
/// @param width Output: parsed field width (or -1 if not present).
/// @param precision Output: parsed precision (or -1 if not present).
/// @param spec Output: the format specifier character.
/// @return Index into fmt right after the specifier.
std::size_t parse_spec(const std::string& fmt, std::size_t pos, bool& zero_pad, int& width, int& precision, char& spec);

/// @brief Base case: no more args; copy remainder, handling '%%' escapes.
/// @param out Destination string.
/// @param fmt The format string.
/// @param start Start index within fmt to continue copying from.
void format_impl(std::string& out, const std::string& fmt, std::size_t start);

template <typename T, typename... Rest>
inline void format_impl(std::string& out, const std::string& fmt, std::size_t start, const T& value, const Rest&... rest) {
  std::size_t i = start;
  while (i < fmt.size()) {
    if (fmt[i] == '%') {
      if (i + 1 < fmt.size() && fmt[i + 1] == '%') {
        out.push_back('%');
        i += 2;
        continue;
      }
      bool zero_pad = false;
      int width = -1;
      int precision = -1;
      char spec = '\0';
      std::size_t next = parse_spec(fmt, i + 1, zero_pad, width, precision, spec);
      if (spec == '\0') {
        // Malformed at end, write '%' and stop
        out.push_back('%');
        format_impl(out, fmt, i + 1, rest...);
        return;
      }
      // Apply formatting based on spec
      std::ostringstream oss;
      switch (spec) {
        case 'd':
        case 'i':
        case 'u':
        case 'f':
        case 'g':
        case 'e':
        case 'E':
        case 's':
          stream_value(oss, spec, width, zero_pad, precision, value);
          break;
        default:
          // Unrecognized specifier: output value using stream
          stream_value(oss, spec, width, zero_pad, precision, value);
          break;
      }
      out += oss.str();
      format_impl(out, fmt, next, rest...);
      return;
    } else {
      out.push_back(fmt[i++]);
    }
  }
  // If we got here, string had no more specifiers; ignore extra args
}

}  // namespace detail

/// @brief Format a string using a small printf-like subset.
/// @param fmt Format string with % directives.
/// @tparam Args Variadic argument types.
/// @param args Arguments referenced by the format string in their given order.
/// @return The formatted string.
template <typename... Args>
inline std::string format(const std::string& fmt, const Args&... args) {
  std::string out;
  out.reserve(fmt.size() + sizeof...(Args) * 8);
  detail::format_impl(out, fmt, 0, args...);
  return out;
}

}  // namespace utility
