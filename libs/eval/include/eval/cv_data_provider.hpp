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
  FileCVDataProvider(const std::string& name) : CVDataProvider(name) {}
  FileCVDataProvider(const std::filesystem::path& p, const std::string& name, bool recursive = true)
      : CVDataProvider(name), pos_(0) {
    parse(p, recursive);
  }
  FileCVDataProvider(const std::vector<std::filesystem::path>& f, const std::string& name, bool recursive = true)
      : CVDataProvider(name), pos_(0) {
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
  size_t pos_;
  std::vector<std::filesystem::path> files_;
};

}  // namespace lsfm
