#pragma once

#include <any>
#include <map>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Component-based metadata container
///
/// Uses std::any for type-erased storage of arbitrary metadata.
/// This allows flexible extension without modifying the base class.
using MetadataMap = std::map<std::string, std::any>;

/// @brief Base measure class with component-based metadata
///
/// Design principles:
/// - Generic base without domain-specific fields (no width/height here)
/// - Component-based metadata for flexible extension
/// - Easy to serialize/deserialize
/// - Python-binding friendly through simple types
struct Measure {
  virtual ~Measure() = default;

  /// Name of source (image/file/dataset)
  std::string source_name{};

  /// Name of task that produced this measure
  std::string task_name{};

  /// Component-based metadata container
  /// Use this to add domain-specific data:
  /// - CV: "width", "height" as double
  /// - Network: "host", "port"
  /// - Custom: any key-value pairs
  MetadataMap metadata{};

  /// @brief Clear all data
  virtual void clear() { metadata.clear(); }

  /// @brief Get metadata value with type
  template <typename T>
  T get_metadata(const std::string& key, const T& default_value = T{}) const {
    auto it = metadata.find(key);
    if (it != metadata.end()) {
      try {
        return std::any_cast<T>(it->second);
      } catch (const std::bad_any_cast&) {
        return default_value;
      }
    }
    /// @file measure.hpp
    /// @brief Base measurement structure for evaluation framework.
    /// Provides generic measurement container with metadata support for extensibility.

    return default_value;
  }

  /// @brief Set metadata value
  template <typename T>
  void set_metadata(const std::string& key, const T& value) {
    metadata[key] = value;
  }

  /// @brief Check if metadata key exists
  bool has_metadata(const std::string& key) const { return metadata.find(key) != metadata.end(); }
};

/// @brief Container for multiple measures
using MeasureVector = std::vector<Measure>;

/// @brief Map of measures by source name
using MeasuresMap = std::map<std::string, Measure>;

}  // namespace lsfm
