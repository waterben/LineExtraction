//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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

}  // namespace lsfm
