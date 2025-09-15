#include <utility/format.hpp>

namespace utility {
namespace detail {

void append_literal(std::string& out, const std::string& fmt, std::size_t start, std::size_t end) {
  if (end > start) out.append(fmt, start, end - start);
}

void stream_value(std::ostringstream& oss, char /*spec*/, int width, bool zero_pad, int precision, const char* value) {
  if (width > 0) {
    oss << std::setw(width) << (zero_pad ? std::setfill('0') : std::setfill(' '));
  }
  if (precision >= 0) {
    oss << std::setprecision(precision);
  }
  oss << value;
}

std::size_t parse_spec(const std::string& fmt, std::size_t pos, bool& zero_pad, int& width, int& precision, char& spec) {
  zero_pad = false;
  width = -1;
  precision = -1;
  std::size_t i = pos;
  if (i < fmt.size() && fmt[i] == '0') {
    zero_pad = true;
    ++i;
  }
  // width
  std::size_t wstart = i;
  while (i < fmt.size() && std::isdigit(static_cast<unsigned char>(fmt[i]))) ++i;
  if (i > wstart) width = std::stoi(fmt.substr(wstart, i - wstart));
  // precision
  if (i < fmt.size() && fmt[i] == '.') {
    ++i;
    std::size_t pstart = i;
    while (i < fmt.size() && std::isdigit(static_cast<unsigned char>(fmt[i]))) ++i;
    if (i > pstart) precision = std::stoi(fmt.substr(pstart, i - pstart));
    else precision = 0;
  }
  // specifier
  if (i < fmt.size()) spec = fmt[i++];
  else spec = '\0';
  return i;
}

void format_impl(std::string& out, const std::string& fmt, std::size_t start) {
  for (std::size_t i = start; i < fmt.size(); ++i) {
    if (fmt[i] == '%' && i + 1 < fmt.size() && fmt[i + 1] == '%') {
      out.push_back('%');
      ++i;
    } else {
      out.push_back(fmt[i]);
    }
  }
}

}  // namespace detail
}  // namespace utility

