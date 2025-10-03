#include <eval/cv_data_provider.hpp>

#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;

namespace lsfm {

void FileCVDataProvider::parse(const fs::path& folder, bool recursive) {
  std::for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&, this](const fs::path& file) {
    if (fs::is_regular_file(file)) {
      std::string ext = file.extension().generic_string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
      if (ext == ".jpg" || ext == ".png") {
        this->files_.push_back(file);
      }
    }
    if (fs::is_directory(file) && recursive) this->parse(file, recursive);
  });
}

bool FileCVDataProvider::get(CVData& data) {
  if (pos_ >= files_.size()) return false;
  fs::path file = files_[pos_++];
  data.name = file.filename().generic_string();
  data.src = cv::imread(file.generic_string());

  if (data.src.empty()) {
    std::cout << "Can not open " << file.generic_string() << std::endl;
    return false;
  }

  return true;
}

}  // namespace lsfm
