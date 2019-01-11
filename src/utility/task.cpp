#include <utility/task.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <fstream>

namespace fs = boost::filesystem;

namespace lsfm {

    void FileDataProvider::parse(const fs::path& folder, bool recursive) {
        std::for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&, this](const fs::path& file) {
            if (fs::is_regular_file(file)) {
                std::string ext = file.extension().generic_string();
                boost::algorithm::to_lower(ext);
                if (ext == ".jpg" || ext == ".png") {
                    this->files_.push_back(file);
                }
            }
            if (fs::is_directory(file) && recursive)
                this->parse(file, recursive);
        });
    }

    bool FileDataProvider::get(std::string& src_name, cv::Mat& src) {
        if (pos_ >= files_.size())
            return false;
        fs::path file = files_[pos_++];
        src_name = file.filename().generic_string();
        src = cv::imread(file.generic_string());

        if (src.empty()) {
            std::cout << "Can not open " << file.generic_string() << std::endl;
            return false;
        }

        return true;
    }

}

