#pragma once

#include <memory>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Generic data provider interface
template <typename DataT>
struct DataProvider {
  DataProvider(const std::string& data_name) : name(data_name) {}
  virtual ~DataProvider() {}

  using DataType = DataT;

  /// @brief Get next image data, return false if source is depleted
  virtual bool get(DataType& data) = 0;
  /// @brief Rewind provider to initial data
  virtual void rewind() = 0;
  /// @brief Clear data
  virtual void clear() = 0;

  /// Name of data provider
  const std::string name{};

  using Ptr = std::shared_ptr<DataProvider>;
  using PtrList = std::vector<Ptr>;
};

}  // namespace lsfm
