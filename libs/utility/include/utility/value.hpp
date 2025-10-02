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

#ifndef _VALUE_HPP_
#define _VALUE_HPP_
#ifdef __cplusplus

#  include <opencv2/opencv.hpp>

#  include <cstring>
#  include <iostream>
#  include <sstream>


namespace lsfm {

class Value {
  union Data {
    double fval;
    int64_t ival;
    char* str;
  };

  Data data_;

 public:
  enum Type { NOT_A_VALUE = 0, FLOAT, INT, BOOL, STRING };

 private:
  Type type_;

  void release() { delete[] data_.str; }

  void create(const char* data) {
    if (type_ == STRING)
      release();
    else
      type_ = STRING;
    if (data == 0) {
      data_.str = new char[1];
      data_.str[0] = 0;
    } else {
      size_t s = std::strlen(data);
      data_.str = new char[s + 1];
      data_.str[s + 1] = 0;
      strcpy(data_.str, data);
    }
  }

  Value(Type t) : type_(t) { data_.fval = 0; }

 public:
  static const Value& NAV() {
    static Value nav(NOT_A_VALUE);
    return nav;
  }

  Value(double fval = 0) : type_(FLOAT) { data_.fval = fval; }
  Value(float fval) : type_(FLOAT) { data_.fval = fval; }
  Value(int64_t ival) : type_(INT) { data_.ival = ival; }
  Value(int ival) : type_(INT) { data_.ival = ival; }
  Value(unsigned int ival) : type_(INT) { data_.ival = ival; }
  Value(short ival) : type_(INT) { data_.ival = ival; }
  Value(unsigned short ival) : type_(INT) { data_.ival = ival; }
  Value(char ival) : type_(INT) { data_.ival = ival; }
  Value(unsigned char ival) : type_(INT) { data_.ival = ival; }
  Value(bool ival) : type_(BOOL) { data_.ival = ival ? 1 : 0; }
  Value(const std::string& str) : type_(INT) { create(str.c_str()); }
  Value(const char* str) : type_(INT) { create(str); }

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

  Type type() const { return type_; }

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

  template <class T>
  T get() const {
    CV_Assert(false);
  }

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
        os << (getInt64() ? "true" : "false");
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

  inline void setInt64(int64_t val) { this->operator=(val); }

  inline void setInt(int val) { this->operator=(val); }

  inline void setFloat(float val) { this->operator=(val); }

  inline void setDouble(double val) { this->operator=(val); }

  inline void setBool(bool val) { this->operator=(val); }

  inline void setString(const std::string& val) { this->operator=(val); }

  inline void setChar(const char* val) { this->operator=(val); }

  inline operator std::string() { return getString(); }

  inline operator const char*() { return getChar(); }

  inline int64_t getInt64() const {
    CV_Assert(type_ == INT);
    return data_.ival;
  }

  inline int getInt() const {
    CV_Assert(type_ == INT);
    return static_cast<int>(data_.ival);
  }

  inline float getFloat() const {
    CV_Assert(type_ == FLOAT);
    return static_cast<float>(data_.fval);
  }

  inline double getDouble() const {
    CV_Assert(type_ == FLOAT);
    return data_.fval;
  }

  inline bool getBool() const {
    CV_Assert(type_ == BOOL);
    return static_cast<bool>(data_.ival != 0);
  }

  inline std::string getString() const {
    CV_Assert(type_ == STRING);
    return data_.str;
  }

  inline const char* getChar() const {
    CV_Assert(type_ == STRING);
    return data_.str;
  }


  inline operator double() const {
    CV_Assert(type_ != STRING);
    return type_ == INT ? static_cast<double>(data_.ival) : data_.fval;
  }

  inline operator float() const {
    CV_Assert(type_ != STRING);
    return type_ == INT ? static_cast<float>(data_.ival) : static_cast<float>(data_.fval);
  }

  inline operator int64_t() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<int64_t>(data_.fval) : data_.ival;
  }

  inline operator uint64_t() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<uint64_t>(data_.fval) : static_cast<uint64_t>(data_.ival);
  }

  inline operator int() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<int>(data_.fval) : static_cast<int>(data_.ival);
  }

  inline operator unsigned int() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<unsigned int>(data_.fval) : static_cast<unsigned int>(data_.ival);
  }

  inline operator short() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<short>(data_.fval) : static_cast<short>(data_.ival);
  }

  inline operator unsigned short() const {
    CV_Assert(type_ != STRING);
    return type_ == FLOAT ? static_cast<unsigned short>(data_.fval) : static_cast<unsigned short>(data_.ival);
  }

  inline operator bool() const {
    return type_ == FLOAT                    ? data_.fval != 0
           : (type_ == INT || type_ == BOOL) ? data_.ival != 0
           : type_ == STRING                 ? data_.str != 0
                                             : false;
  }
};

template <>
inline std::string Value::get<std::string>() const {
  return getString();
}

template <>
inline const char* Value::get<const char*>() const {
  return getChar();
}


template <>
inline double Value::get<double>() const {
  return getDouble();
}

template <>
inline float Value::get<float>() const {
  return static_cast<float>(getFloat());
}

template <>
inline int64_t Value::get<int64_t>() const {
  return getInt64();
}

template <>
inline uint64_t Value::get<uint64_t>() const {
  return static_cast<uint64_t>(getInt64());
}

template <>
inline int Value::get<int>() const {
  return getInt();
}

template <>
inline unsigned int Value::get<unsigned int>() const {
  return static_cast<unsigned int>(getInt64());
}

template <>
inline short Value::get<short>() const {
  return static_cast<short>(getInt());
}

template <>
inline unsigned short Value::get<unsigned short>() const {
  return static_cast<unsigned short>(getInt());
}

template <>
inline bool Value::get<bool>() const {
  return getBool();
}

inline std::ostream& operator<<(std::ostream& os, const Value& v) {
  os << v.toString();
  return os;
}

inline std::istream& operator>>(std::istream& is, Value& v) {
  std::string tmp;
  is >> tmp;
  v.fromString(tmp);
  return is;
}

}  // namespace lsfm
#endif
#endif
