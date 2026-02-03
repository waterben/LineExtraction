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

//! Image data set for performance test
struct FileCVDataProvider : public CVDataProvider {
  FileCVDataProvider(const std::string& provider_name) : CVDataProvider(provider_name) {}
  FileCVDataProvider(const std::filesystem::path& p, const std::string& provider_name, bool recursive = true)
      : CVDataProvider(provider_name) {
    parse(p, recursive);
  }
  FileCVDataProvider(const std::vector<std::filesystem::path>& f,
                     const std::string& provider_name,
                     bool recursive = true)
      : CVDataProvider(provider_name) {
    parse(f, recursive);
  }

  virtual ~FileCVDataProvider() = default;

  void parse(const std::vector<std::filesystem::path>& folders, bool recursive = true) {
    for_each(folders.begin(), folders.end(), [&, this](const std::filesystem::path& f) { this->parse(f, recursive); });
  }

  void parse(const std::filesystem::path& folder, bool recursive = true);

  //! get next image data, return false if source is depleted
  bool get(CVData& data) override;

  //! rewind provider to initial data
  void rewind() override { pos_ = 0; }

  void clear() override {
    files_.clear();
    pos_ = 0;
  }

 private:
  size_t pos_{0};
  std::vector<std::filesystem::path> files_{};
};

}  // namespace lsfm
