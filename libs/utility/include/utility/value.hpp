/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/// @file value.hpp
/// @brief Type-safe variant class for storing different value types.

#pragma once

#include <opencv2/core.hpp>

#include <cstring>
#include <iostream>
#include <sstream>


namespace lsfm {

/// @brief Type-safe variant class supporting float, int, bool, and string values.
///
/// Value provides dynamic typing for algorithm parameters, supporting automatic
/// type conversion and serialization. Used extensively by ValueManager for
/// runtime parameter configuration.
class Value {
  /// @brief Internal storage union for different value types.
  union Data {
    double fval;   ///< Storage for floating-point values.
    int64_t ival;  ///< Storage for integer/bool values.
    char* str;     ///< Storage for string values (heap-allocated).

    Data() : str(nullptr) {}
  };

  Data data_;  ///< Internal data storage.

 public:
  /// @brief Supported value types.
  enum Type {
    NOT_A_VALUE = 0,  ///< Invalid/unset value.
    FLOAT,            ///< Floating-point (double precision).
    INT,              ///< 64-bit integer.
    BOOL,             ///< Boolean (stored as int).
    STRING            ///< String (heap-allocated).
  };

 private:
  Type type_{NOT_A_VALUE};  ///< Current value type.

  void release() { delete[] data_.str; }

  void create(const char* data) {
    if (type_ == STRING)
      release();
    else
      type_ = STRING;
    if (data == nullptr) {
      data_.str = new char[1];
      data_.str[0] = 0;
    } else {
      size_t s = std::strlen(data);
      data_.str = new char[s + 1];
      data_.str[s] = 0;
      strcpy(data_.str, data);
    }
  }

  Value(Type t) : data_(), type_(t) { data_.fval = 0; }

 public:
  /// @brief Get singleton "Not A Value" instance.
  /// @return Reference to static NAV instance.
  static const Value& NAV() {
    static Value nav(NOT_A_VALUE);
    return nav;
  }

  /// @brief Construct from double (FLOAT type).
  /// @param fval Double value.
  Value(double fval = 0) : data_(), type_(FLOAT) { data_.fval = fval; }

  /// @brief Construct from float (FLOAT type).
  /// @param fval Float value.
  Value(float fval) : data_(), type_(FLOAT) { data_.fval = fval; }

  /// @brief Construct from int64 (INT type).
  /// @param ival Integer value.
  Value(int64_t ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from int (INT type).
  /// @param ival Integer value.
  Value(int ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from unsigned int (INT type).
  /// @param ival Unsigned integer value.
  Value(unsigned int ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from short (INT type).
  /// @param ival Short integer value.
  Value(short ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from unsigned short (INT type).
  /// @param ival Unsigned short integer value.
  Value(unsigned short ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from char (INT type).
  /// @param ival Character value.
  Value(char ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from unsigned char (INT type).
  /// @param ival Unsigned character value.
  Value(unsigned char ival) : data_(), type_(INT) { data_.ival = ival; }

  /// @brief Construct from bool (BOOL type).
  /// @param ival Boolean value.
  Value(bool ival) : data_(), type_(BOOL) { data_.ival = ival ? 1 : 0; }

  /// @brief Construct from std::string (STRING type).
  /// @param str String value.
  Value(const std::string& str) : data_(), type_(INT) { create(str.c_str()); }

  /// @brief Construct from C-string (STRING type).
  /// @param str C-string value.
  Value(const char* str) : data_(), type_(INT) { create(str); }

  /// @brief Copy constructor.
  /// @param val Value to copy.
  Value(const Value& val) : data_(val.data_), type_(val.type_) {
    if (type_ == STRING) {
      // prevent create to release data at pointer
      type_ = INT;
      create(val.data_.str);
    }
  }

  Value& operator=(double fval) {
    if (type_ == STRING) release();
    type_ = FLOAT;
    data_.fval = fval;
    return *this;
  }

  Value& operator=(float fval) {
    if (type_ == STRING) release();
    type_ = FLOAT;
    data_.fval = static_cast<double>(fval);
    return *this;
  }

  Value& operator=(int64_t ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = ival;
    return *this;
  }

  Value& operator=(int ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = static_cast<int64_t>(ival);
    return *this;
  }

  Value& operator=(unsigned int ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = static_cast<int64_t>(ival);
    return *this;
  }
  Value& operator=(short ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = static_cast<int64_t>(ival);
    return *this;
  }

  Value& operator=(unsigned short ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = static_cast<int64_t>(ival);
    return *this;
  }

  Value& operator=(bool ival) {
    if (type_ == STRING) release();
    type_ = INT;
    data_.ival = ival ? 1 : 0;
    return *this;
  }

  Value& operator=(const std::string& str) {
    create(str.c_str());
    return *this;
  }

  Value& operator=(const char* str) {
    create(str);
    return *this;
  }

  Value& operator=(const Value& val) {
    if (val.type_ == STRING) {
      create(val.data_.str);
    } else {
      if (type_ == STRING) release();
      type_ = val.type_;
      data_ = val.data_;
    }
    return *this;
  }


  ~Value() {
    if (type_ == STRING) release();
  }

  /// @brief Get the current value type.
  /// @return The type enum.
  Type type() const { return type_; }

  /// @brief Get human-readable type name.
  /// @return Type name string ("float", "int", "bool", "string", "not_a_value").
  std::string typeName() const {
    switch (type_) {
      case FLOAT:
        return "float";
      case INT:
        return "int";
      case BOOL:
        return "bool";
      case STRING:
        return "string";
      case NOT_A_VALUE:
        return "not_a_value";
    }
    return "unknown";
  }

  /// @brief Get value as specified type.
  /// @tparam T Target type (specialized for common types).
  /// @return Value converted to T.
  template <class T>
  T get() const {
    CV_Assert(false);
  }

  /// @brief Convert value to string representation.
  /// @return String representation of current value.
  std::string toString() const {
    std::ostringstream os;
    switch (type()) {
      case Value::FLOAT:
        os << getDouble();
        break;
      case Value::INT:
        os << getInt64();
        break;
      case Value::BOOL:
        os << (getBool() ? "true" : "false");
        break;
      case Value::STRING:
        os << getChar();
        break;
      case Value::NOT_A_VALUE:
        os << "nav";
        break;
    }
    return os.str();
  }

  /// @brief Parse value from string with auto type detection.
  /// @param in String to parse.
  void fromString(const std::string& in) {
    std::istringstream is(in);
    // try to read double
    if (in.find_first_of('.') != std::string::npos) {
      double tmp;
      is >> tmp;
      if (is.fail())
        this->operator=(in);
      else
        this->operator=(tmp);
    } else {
      int64_t tmp;
      is >> tmp;
      if (is.fail()) {
        if (in == "true")
          this->operator=(true);
        else if (in == "false")
          this->operator=(false);
        else
          this->operator=(in);
      } else
        this->operator=(tmp);
    }
  }

  /// @brief Set value as 64-bit integer.
  inline void setInt64(int64_t val) { this->operator=(val); }

  /// @brief Set value as integer.
  inline void setInt(int val) { this->operator=(val); }

  /// @brief Set value as float.
  inline void setFloat(float val) { this->operator=(val); }

  /// @brief Set value as double.
  inline void setDouble(double val) { this->operator=(val); }

  /// @brief Set value as boolean.
  inline void setBool(bool val) { this->operator=(val); }

  /// @brief Set value as string.
  inline void setString(const std::string& val) { this->operator=(val); }

  /// @brief Set value as C-string.
  inline void setChar(const char* val) { this->operator=(val); }

  /// @brief Convert to std::string.
  inline operator std::string() { return getString(); }

  /// @brief Convert to C-string.
  inline operator const char*() { return getChar(); }

  /// @brief Get value as 64-bit integer.
  /// @return Integer value.
  /// @throws cv::Exception if type is not INT.
  inline int64_t getInt64() const {
    CV_Assert(type_ == INT);
    return data_.ival;
  }

  /// @brief Get value as integer.
  /// @return Integer value.
  /// @throws cv::Exception if type is not INT.
  inline int getInt() const {
    CV_Assert(type_ == INT);
    return static_cast<int>(data_.ival);
  }

  /// @brief Get value as float.
  /// @return Float value.
  /// @throws cv::Exception if type is not FLOAT.
  inline float getFloat() const {
    CV_Assert(type_ == FLOAT);
    return static_cast<float>(data_.fval);
  }

  /// @brief Get value as double.
  /// @return Double value.
  /// @throws cv::Exception if type is not FLOAT.
  inline double getDouble() const {
    CV_Assert(type_ == FLOAT);
    return data_.fval;
  }

  /// @brief Get value as boolean.
  /// @return Boolean value.
  /// @throws cv::Exception if type is not BOOL.
  inline bool getBool() const {
    CV_Assert(type_ == BOOL);
    return static_cast<bool>(data_.ival != 0);
  }

  /// @brief Get value as std::string.
  /// @return String value.
  /// @throws cv::Exception if type is not STRING.
  inline std::string getString() const {
    CV_Assert(type_ == STRING);
    return data_.str;
  }

  /// @brief Get value as C-string.
  /// @return Pointer to internal string buffer.
  /// @throws cv::Exception if type is not STRING.
  inline const char* getChar() const {
    CV_Assert(type_ == STRING);
    return data_.str;
  }

  /// @brief Implicit conversion to double.
  inline operator double() const {
    CV_Assert(type_ != STRING);
    return type_ == INT ? static_cast<double>(data_.ival) : data_.fval;
  }

  /// @brief Implicit conversion to float.
  inline operator float() const {
    CV_Assert(type_ != STRING);
    return type_ == INT ? static_cast<float>(data_.ival) : static_cast<float>(data_.fval);
  }

  /// @brief Implicit conversion to 64-bit signed integer.
  inline operator int64_t() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<int64_t>(data_.fval) : data_.ival;
  }

  /// @brief Implicit conversion to 64-bit unsigned integer.
  inline operator uint64_t() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<uint64_t>(data_.fval) : static_cast<uint64_t>(data_.ival);
  }

  /// @brief Implicit conversion to int.
  inline operator int() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<int>(data_.fval) : static_cast<int>(data_.ival);
  }

  /// @brief Implicit conversion to unsigned int.
  inline operator unsigned int() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<unsigned int>(data_.fval) : static_cast<unsigned int>(data_.ival);
  }

  /// @brief Implicit conversion to short.
  inline operator short() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<short>(data_.fval) : static_cast<short>(data_.ival);
  }

  /// @brief Implicit conversion to unsigned short.
  inline operator unsigned short() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<unsigned short>(data_.fval) : static_cast<unsigned short>(data_.ival);
  }

  /// @brief Implicit conversion to bool.
  inline operator bool() const {
    return type_ == FLOAT                    ? data_.fval != 0
           : (type_ == INT || type_ == BOOL) ? data_.ival != 0
           : type_ == STRING                 ? data_.str != nullptr
                                             : false;
  }
};

/// @brief Template specialization to get value as std::string.
template <>
inline std::string Value::get<std::string>() const {
  return getString();
}

/// @brief Template specialization to get value as const char*.
template <>
inline const char* Value::get<const char*>() const {
  return getChar();
}

/// @brief Template specialization to get value as double.
template <>
inline double Value::get<double>() const {
  return getDouble();
}

/// @brief Template specialization to get value as float.
template <>
inline float Value::get<float>() const {
  return static_cast<float>(getFloat());
}

/// @brief Template specialization to get value as int64_t.
template <>
inline int64_t Value::get<int64_t>() const {
  return getInt64();
}

/// @brief Template specialization to get value as uint64_t.
template <>
inline uint64_t Value::get<uint64_t>() const {
  return static_cast<uint64_t>(getInt64());
}

/// @brief Template specialization to get value as int.
template <>
inline int Value::get<int>() const {
  return getInt();
}

/// @brief Template specialization to get value as unsigned int.
template <>
inline unsigned int Value::get<unsigned int>() const {
  return static_cast<unsigned int>(getInt64());
}

/// @brief Template specialization to get value as short.
template <>
inline short Value::get<short>() const {
  return static_cast<short>(getInt());
}

/// @brief Template specialization to get value as unsigned short.
template <>
inline unsigned short Value::get<unsigned short>() const {
  return static_cast<unsigned short>(getInt());
}

/// @brief Template specialization to get value as bool.
template <>
inline bool Value::get<bool>() const {
  return getBool();
}

/// @brief Stream output operator for Value.
/// @param os Output stream.
/// @param v Value to output.
/// @return Reference to the output stream.
inline std::ostream& operator<<(std::ostream& os, const Value& v) {
  os << v.toString();
  return os;
}

/// @brief Stream input operator for Value.
/// @param is Input stream.
/// @param v Value to populate.
/// @return Reference to the input stream.
inline std::istream& operator>>(std::istream& is, Value& v) {
  std::string tmp;
  is >> tmp;
  v.fromString(tmp);
  return is;
}

}  // namespace lsfm
