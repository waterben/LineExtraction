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

/// @file option_manager.hpp
/// @brief Legacy option management interface (prefer ValueManager).

#pragma once

#include <algorithm>
#include <string>
#include <vector>


namespace lsfm {

/// @brief Legacy class for managing algorithm options.
///
/// OptionManager provides a simple interface for storing and retrieving
/// algorithm parameters as name-value pairs. Values are stored as doubles.
/// @note Prefer ValueManager for new code as it supports multiple types.
class OptionManager {
 public:
  /// @brief Container for a single option's metadata and value.
  struct OptionEntry {
    std::string name{};         ///< Option name (key).
    double value{0.0};          ///< Option value (stored as double).
    std::string type{};         ///< Type hint string (e.g., "int", "float").
    std::string description{};  ///< Human-readable description.

    OptionEntry() = default;

    /// @brief Construct with all fields.
    /// @param n Option name.
    /// @param v Option value.
    /// @param t Type hint.
    /// @param d Description.
    OptionEntry(std::string n, double v, std::string t, std::string d)
        : name(std::move(n)), value(v), type(std::move(t)), description(std::move(d)) {}

    /// @brief Get value cast to specified type.
    /// @tparam T Target type.
    /// @return Value cast to T.
    template <typename T>
    T get() const {
      return static_cast<T>(value);
    }

    /// @brief Set value from specified type.
    /// @tparam T Source type.
    /// @param val Value to set.
    template <typename T>
    void set(T val) {
      value = static_cast<double>(val);
    }
  };

  typedef std::vector<OptionEntry> OptionVector;  ///< Vector of option entries.

  /// @brief Get all registered options.
  /// @return Const reference to option vector.
  inline const OptionVector& getOptions() const { return options_; }

  /// @brief Get option by name.
  /// @param name Option name to find.
  /// @return OptionEntry or empty entry if not found.
  OptionEntry getOption(const std::string& name) const {
    OptionVector::const_iterator f =
        std::find_if(options_.begin(), options_.end(), [&name](const OptionEntry& e) { return (e.name == name); });
    return f != options_.end() ? *f : OptionEntry();
  }

  /// @brief Set option value by name.
  /// @param name Option name.
  /// @param value New value.
  inline void setOption(const std::string& name, double value) { setOptionImpl(name, value); }

  /// @brief Set option from OptionEntry.
  /// @param option Entry containing name and value.
  inline void setOption(const OptionEntry& option) { setOption(option.name, option.value); }

  /// @brief Set multiple options from vector.
  /// @param options Vector of entries to set.
  inline void setOptions(const OptionVector& options) {
    for_each(options.begin(), options.end(), [this](const OptionEntry& e) { setOption(e.name, e.value); });
  }

  virtual ~OptionManager() = default;

 protected:
  /// @brief Construct with initial options.
  /// @param opv Initial option vector.
  OptionManager(const OptionVector& opv = OptionVector()) : options_(opv) {}

  OptionVector options_{};  ///< Storage for all options.

  /// @brief Override to handle option changes.
  /// @param name Option name being set.
  /// @param value New value.
  virtual void setOptionImpl(const std::string& /*name*/, double /*value*/) {};
};
};

}  // namespace lsfm
