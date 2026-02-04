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

/// @file value_manager.hpp
/// @brief Runtime parameter management via name-value pairs.

#pragma once

#include <utility/value.hpp>

#include <functional>
#include <initializer_list>
#include <map>
#include <vector>


namespace lsfm {

/// @brief Base class for managing runtime-configurable parameters.
///
/// ValueManager provides a generic interface for setting and getting algorithm
/// parameters by name. Classes inherit from ValueManager and register their
/// parameters via functors that handle get/set operations.
class ValueManager {
 protected:
  /// @brief Functor type for get/set operations.
  typedef std::function<Value(const Value& v)> Functor;

  /// @brief Internal entry storing a parameter's functor and description.
  struct ValueEntry {
    std::string description{};  ///< Human-readable parameter description.
    Functor func{};             ///< Functor for get (NAV input) or set operations.

    ValueEntry() = default;

    /// @brief Construct with functor and optional description.
    /// @param f Functor handling get/set.
    /// @param dsc Parameter description.
    ValueEntry(Functor f, std::string dsc = std::string()) : description(std::move(dsc)), func(std::move(f)) {}
  };

  typedef std::map<std::string, ValueEntry> ValueMap;
  ValueMap values_{};

  ValueManager() {}

  ValueManager(const ValueManager& om) : values_(om.values_) {}

  ValueManager(ValueManager&& om) noexcept : values_(std::move(om.values_)) {}

  ValueManager& operator=(const ValueManager& om) {
    if (this != &om) {
      values_ = om.values_;
    }
    return *this;
  }

  ValueManager& operator=(ValueManager&& om) noexcept {
    if (this != &om) {
      values_ = std::move(om.values_);
    }
    return *this;
  }

  /// @brief Register a new parameter.
  /// @param name Parameter name (key).
  /// @param func Functor: returns current value when called with NAV, sets when called with value.
  /// @param dsc Optional description.
  void add(const std::string& name, const Functor& func, const std::string& dsc = std::string()) {
    values_[name] = ValueEntry(func, dsc);
  }

  /// @brief Merge parameters from another ValueManager.
  /// @tparam VM Type derived from ValueManager.
  /// @param vm Manager whose parameters to add.
  template <class VM>
  void addManager(const VM& vm) {
    const ValueManager* tmp = dynamic_cast<const ValueManager*>(&vm);
    if (tmp) addManager(*tmp);
  }


 public:
  virtual ~ValueManager() = default;

  /// @brief Container for parameter name, value, and description.
  struct NameValuePair {
    std::string name{};         ///< Parameter name.
    std::string description{};  ///< Parameter description.
    Value value{};              ///< Current parameter value.

    NameValuePair() = default;

    /// @brief Construct with name, value, and optional description.
    NameValuePair(std::string n, Value v, std::string d = std::string())
        : name(std::move(n)), description(std::move(d)), value(std::move(v)) {}
  };


  typedef std::vector<NameValuePair> NameValueVector;            ///< Vector of name-value pairs.
  typedef std::initializer_list<NameValuePair> InitializerList;  ///< Initializer list type.

  /// @brief Get all registered parameters.
  /// @return Vector of all name-value pairs.
  inline NameValueVector values() const {
    NameValueVector ret;
    for_each(values_.begin(), values_.end(), [&ret](const std::pair<std::string, ValueEntry>& e) {
      ret.push_back(NameValuePair(e.first, e.second.func(Value::NAV()), e.second.description));
    });
    return ret;
  }

  /// @brief Get parameter by name as NameValuePair.
  /// @param name Parameter name.
  /// @return NameValuePair or empty if not found.
  NameValuePair valuePair(const std::string& name) const {
    ValueMap::const_iterator f = values_.find(name);
    if (f != values_.end()) return NameValuePair(f->first, f->second.func(Value::NAV()), f->second.description);
    return NameValuePair();
  }

  /// @brief Get parameter by index as NameValuePair.
  /// @param idx Parameter index.
  /// @return NameValuePair or empty if index out of range.
  NameValuePair valuePair(size_t idx) const {
    if (idx > values_.size()) return NameValuePair();
    ValueMap::const_iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    return NameValuePair(f->first, f->second.func(Value::NAV()), f->second.description);
  }

  /// @brief Get parameter value by name.
  /// @param name Parameter name.
  /// @return Value or empty Value if not found.
  Value value(const std::string& name) const {
    ValueMap::const_iterator f = values_.find(name);
    if (f != values_.end()) return f->second.func(Value::NAV());
    return Value();
  }

  /// @brief Get parameter value by index.
  /// @param idx Parameter index.
  /// @return Value or empty Value if index out of range.
  Value value(size_t idx) const {
    if (idx > values_.size()) return Value();
    ValueMap::const_iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    return f->second.func(Value::NAV());
  }

  /// @brief Set parameter value by name.
  /// @param name Parameter name.
  /// @param value New value to set.
  inline void value(const std::string& name, const Value& value) {
    ValueMap::iterator f = values_.find(name);
    if (f != values_.end()) f->second.func(value);
  }

  /// @brief Set parameter value by index.
  /// @param idx Parameter index.
  /// @param value New value to set.
  inline void value(size_t idx, const Value& value) {
    if (idx > values_.size()) return;
    ValueMap::iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    f->second.func(value);
  }

  /// @brief Set parameter from NameValuePair.
  /// @param p Name-value pair to set.
  inline void value(const NameValuePair& p) { value(p.name, p.value); }

  /// @brief Set multiple parameters from vector.
  /// @param v Vector of name-value pairs.
  inline void value(const NameValueVector& v) {
    for_each(v.begin(), v.end(), [this](const NameValuePair& p) { this->value(p.name, p.value); });
  }

  /// @brief Set multiple parameters from initializer list.
  /// @param list Initializer list of name-value pairs.
  inline void value(InitializerList list) {
    std::for_each(list.begin(), list.end(), [this](const NameValuePair& p) { this->value(p.name, p.value); });
  }
};

/// @brief Specialization to merge all parameters from another ValueManager.
template <>
inline void ValueManager::addManager<ValueManager>(const ValueManager& vm) {
  for_each(vm.values_.begin(), vm.values_.end(),
           [this](const std::pair<std::string, ValueEntry>& e) { this->values_[e.first] = e.second; });
}

typedef ValueManager::NameValuePair NV;    ///< Shorthand for NameValuePair.
typedef ValueManager::InitializerList IL;  ///< Shorthand for InitializerList.
}  // namespace lsfm
