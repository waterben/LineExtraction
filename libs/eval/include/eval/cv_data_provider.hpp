//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file cv_data_provider.hpp
/// @brief OpenCV-based image data providers.
/// Provides implementations for supplying image data to evaluation tasks.

#pragma once

#include <eval/data_provider.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>

namespace lsfm {

/// @brief OpenCV Data provider (read images and provide as mat)
struct CVData {
  std::string name{};
  cv::Mat src{};
};

using CVDataProvider = DataProvider<CVData>;

/// @brief File-based OpenCV data provider that reads images from the filesystem.
/// Scans directories for supported image files and provides them as CVData.
struct FileCVDataProvider : public CVDataProvider {
  /// @brief Construct with a provider name only (no data loaded).
  /// @param provider_name Name identifying this data provider
  FileCVDataProvider(const std::string& provider_name) : CVDataProvider(provider_name) {}

  /// @brief Construct and parse a single directory for images.
  /// @param p Path to the directory to scan
  /// @param provider_name Name identifying this data provider
  /// @param recursive If true, scan subdirectories recursively
  FileCVDataProvider(const std::filesystem::path& p, const std::string& provider_name, bool recursive = true)
      : CVDataProvider(provider_name) {
    parse(p, recursive);
  }

  /// @brief Construct and parse multiple directories for images.
  /// @param f Vector of directory paths to scan
  /// @param provider_name Name identifying this data provider
  /// @param recursive If true, scan subdirectories recursively
  FileCVDataProvider(const std::vector<std::filesystem::path>& f,
                     const std::string& provider_name,
                     bool recursive = true)
      : CVDataProvider(provider_name) {
    parse(f, recursive);
  }

  virtual ~FileCVDataProvider() = default;

  /// @brief Parse multiple directories for image files.
  /// @param folders Vector of directory paths to scan
  /// @param recursive If true, scan subdirectories recursively
  void parse(const std::vector<std::filesystem::path>& folders, bool recursive = true) {
    for_each(folders.begin(), folders.end(), [&, this](const std::filesystem::path& f) { this->parse(f, recursive); });
  }

  /// @brief Parse a single directory for image files.
  /// @param folder Path to the directory to scan
  /// @param recursive If true, scan subdirectories recursively
  void parse(const std::filesystem::path& folder, bool recursive = true);

  /// @brief Get next image data.
  /// @param data Output CVData with loaded image and filename
  /// @return False if source is depleted
  bool get(CVData& data) override;

  /// @brief Rewind provider to the first image.
  void rewind() override { pos_ = 0; }

  /// @brief Clear all loaded file paths and reset position.
  void clear() override {
    files_.clear();
    pos_ = 0;
  }

 private:
  /// @brief Current read position in the file list.
  size_t pos_{0};
  /// @brief List of discovered image file paths.
  std::vector<std::filesystem::path> files_{};
};

}  // namespace lsfm
