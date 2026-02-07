//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file data_provider.hpp
/// @brief Abstract data provider interface for evaluation tasks.
/// Defines generic interface for providing data to evaluation tasks.

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Generic data provider template interface.
/// Provides access to a sequence of data items that can be rewound and cleared.
/// @tparam DataT The type of data provided by this provider
template <typename DataT>
struct DataProvider {
  /// @brief Construct with a provider name.
  /// @param data_name The name of this data provider
  DataProvider(const std::string& data_name) : name(data_name) {}
  virtual ~DataProvider() {}

  using DataType = DataT;  ///< Type alias for provided data

  /// @brief Get next data item.
  /// @param data Output reference to store the next data item
  /// @return True if data was successfully retrieved, false if source is depleted
  virtual bool get(DataType& data) = 0;

  /// @brief Rewind provider to initial state.
  /// Allows iterating over the data again from the beginning.
  virtual void rewind() = 0;

  /// @brief Clear all data.
  virtual void clear() = 0;

  const std::string name{};  ///< Name of this data provider

  using Ptr = std::shared_ptr<DataProvider>;  ///< Smart pointer type
  using PtrList = std::vector<Ptr>;           ///< Vector of smart pointers
};

}  // namespace lsfm
