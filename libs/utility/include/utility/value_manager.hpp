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

#ifndef _VALUE_MANAGER_HPP_
#define _VALUE_MANAGER_HPP_
#ifdef __cplusplus

#  include <utility/value.hpp>

#  include <functional>
#  include <initializer_list>
#  include <map>
#  include <vector>


namespace lsfm {

//! Helper to manage internal setting and states via a generic interface
class ValueManager {
 protected:
  typedef std::function<Value(const Value& v)> Functor;

  struct ValueEntry {
    ValueEntry() {}


    ValueEntry(const Functor& f, const std::string& dsc = std::string()) : func(f), description(dsc) {}

    std::string description;
    Functor func;
  };

  typedef std::map<std::string, ValueEntry> ValueMap;
  ValueMap values_;

  ValueManager() {}

  ValueManager(const ValueManager& om) : values_(om.values_) {}

  // add new option
  void add(const std::string& name, const Functor& func, const std::string& dsc = std::string()) {
    values_[name] = ValueEntry(func, dsc);
  }


  template <class VM>
  void addManager(const VM& vm) {
    const ValueManager* tmp = dynamic_cast<const ValueManager*>(&vm);
    if (tmp) addManager(*tmp);
  }


 public:
  struct NameValuePair {
    NameValuePair() {}
    NameValuePair(const std::string& n, const Value& v, const std::string& d = std::string())
        : name(n), description(d), value(v) {}

    std::string name, description;
    Value value;
  };


  typedef std::vector<NameValuePair> NameValueVector;
  typedef std::initializer_list<NameValuePair> InitializerList;


  //! Get list of all values as name value pairs
  inline NameValueVector values() const {
    NameValueVector ret;
    for_each(values_.begin(), values_.end(), [&ret](const std::pair<std::string, ValueEntry>& e) {
      ret.push_back(NameValuePair(e.first, e.second.func(Value::NAV()), e.second.description));
    });
    return ret;
  }

  //! Get name value pair by name
  NameValuePair valuePair(const std::string& name) const {
    ValueMap::const_iterator f = values_.find(name);
    if (f != values_.end()) return NameValuePair(f->first, f->second.func(Value::NAV()), f->second.description);
    return NameValuePair();
  }

  //! Get value entry by index
  NameValuePair valuePair(size_t idx) const {
    if (idx > values_.size()) return NameValuePair();
    ValueMap::const_iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    return NameValuePair(f->first, f->second.func(Value::NAV()), f->second.description);
  }


  //! Get value entry by name
  Value value(const std::string& name) const {
    ValueMap::const_iterator f = values_.find(name);
    if (f != values_.end()) return f->second.func(Value::NAV());
    return Value();
  }

  //! Get value entry by index
  Value value(size_t idx) const {
    if (idx > values_.size()) return Value();
    ValueMap::const_iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    return f->second.func(Value::NAV());
  }


  //! Set single value by name
  inline void value(const std::string& name, const Value& value) {
    ValueMap::iterator f = values_.find(name);
    if (f != values_.end()) f->second.func(value);
  }

  //! Set single value by idx
  inline void value(size_t idx, const Value& value) {
    if (idx > values_.size()) return;
    ValueMap::iterator f = values_.begin();
    for (size_t i = 0; i != idx; ++i, ++f);
    f->second.func(value);
  }

  //! Set single values by NameValuePair
  inline void value(const NameValuePair& p) { value(p.name, p.value); }

  //! Set multiple values by NameValueVector
  inline void value(const NameValueVector& v) {
    for_each(v.begin(), v.end(), [this](const NameValuePair& p) { this->value(p.name, p.value); });
  }

  //! Set multiple values by NameValueVector
  inline void value(InitializerList list) {
    std::for_each(list.begin(), list.end(), [this](const NameValuePair& p) { this->value(p.name, p.value); });
  }
};

template <>
inline void ValueManager::addManager<ValueManager>(const ValueManager& vm) {
  for_each(vm.values_.begin(), vm.values_.end(),
           [this](const std::pair<std::string, ValueEntry>& e) { this->values_[e.first] = e.second; });
}

typedef ValueManager::NameValuePair NV;
typedef ValueManager::InitializerList IL;
}  // namespace lsfm
#endif
#endif
