#include <eval/cv_data_provider.hpp>

#include <iostream>
#include <algorithm>

namespace fs = std::filesystem;

namespace lsfm {

void FileCVDataProvider::parse(const fs::path& folder, bool recursive) {
  std::for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&, this](const fs::path& file) {
    if (fs::is_regular_file(file)) {
      std::string ext = file.extension().generic_string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
      if (ext == ".jpg" || ext == ".png") {
        this->files_.push_back(file);
      }
    }
    if (fs::is_directory(file) && recursive) this->parse(file, recursive);
  });
}

bool FileCVDataProvider::get(std::string& src_name, cv::Mat& src) {
  if (pos_ >= files_.size()) return false;
  fs::path file = files_[pos_++];
  src_name = file.filename().generic_string();
  src = cv::imread(file.generic_string());

  if (src.empty()) {
    std::cout << "Can not open " << file.generic_string() << std::endl;
    return false;
  }

  return true;
}

}  // namespace lsfm
