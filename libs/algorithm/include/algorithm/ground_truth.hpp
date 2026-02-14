//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file ground_truth.hpp
/// @brief Ground truth line segment loading for benchmark datasets.
///
/// Supports loading ground truth annotations from CSV/JSON format.
/// Designed for datasets like Wireframe (ShanghaiTech) and York Urban DB.

#pragma once

#include <geometry/line.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Ground truth entry for a single image.
struct GroundTruthEntry {
  std::string image_name{};                     ///< Image filename.
  std::vector<LineSegment<double>> segments{};  ///< Ground truth line segments.
};

/// @brief Load ground truth line segments from CSV files.
///
/// CSV format: one line per segment, with columns:
/// `image_name,x1,y1,x2,y2`
///
/// Groups segments by image name into GroundTruthEntry objects.
class GroundTruthLoader {
 public:
  /// @brief Load ground truth from a CSV file.
  /// @param csv_path Path to the CSV file.
  /// @return Vector of ground truth entries, one per image.
  /// @throws std::runtime_error if file cannot be opened.
  static std::vector<GroundTruthEntry> load_csv(const std::string& csv_path) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open ground truth file: " + csv_path);
    }

    std::vector<GroundTruthEntry> result;
    std::string line;
    std::string current_image;
    GroundTruthEntry current_entry;

    // Skip header if present
    if (std::getline(file, line)) {
      if (line.find("image") != std::string::npos || line.find("x1") != std::string::npos) {
        // Header line, skip
      } else {
        // Not a header, parse it
        parse_line(line, current_image, current_entry, result);
      }
    }

    while (std::getline(file, line)) {
      parse_line(line, current_image, current_entry, result);
    }

    // Push last entry
    if (!current_entry.image_name.empty()) {
      result.push_back(std::move(current_entry));
    }

    return result;
  }

  /// @brief Create a ground truth entry from individual line segments.
  /// @param image_name Name of the image.
  /// @param segments Ground truth line segments.
  /// @return GroundTruthEntry.
  static GroundTruthEntry make_entry(const std::string& image_name, const std::vector<LineSegment<double>>& segments) {
    GroundTruthEntry entry;
    entry.image_name = image_name;
    entry.segments = segments;
    return entry;
  }

  /// @brief Save ground truth entries to a CSV file.
  /// @param csv_path Output CSV file path.
  /// @param entries Ground truth entries to save.
  /// @throws std::runtime_error if file cannot be opened.
  static void save_csv(const std::string& csv_path, const std::vector<GroundTruthEntry>& entries) {
    std::ofstream file(csv_path);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open output file: " + csv_path);
    }

    file << "image_name,x1,y1,x2,y2\n";
    for (const auto& entry : entries) {
      for (const auto& seg : entry.segments) {
        auto sp = seg.startPoint();
        auto ep = seg.endPoint();
        file << entry.image_name << "," << getX(sp) << "," << getY(sp) << "," << getX(ep) << "," << getY(ep) << "\n";
      }
    }
  }

 private:
  /// @brief Parse a single CSV line.
  static void parse_line(const std::string& line,
                         std::string& current_image,
                         GroundTruthEntry& current_entry,
                         std::vector<GroundTruthEntry>& result) {
    if (line.empty()) return;

    std::istringstream ss(line);
    std::string image_name;
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    char delim = 0;

    std::getline(ss, image_name, ',');
    ss >> x1 >> delim >> y1 >> delim >> x2 >> delim >> y2;

    if (ss.fail()) return;

    if (image_name != current_image) {
      if (!current_entry.image_name.empty()) {
        result.push_back(std::move(current_entry));
        current_entry = GroundTruthEntry();
      }
      current_image = image_name;
      current_entry.image_name = image_name;
    }

    current_entry.segments.emplace_back(cv::Point2d(x1, y1), cv::Point2d(x2, y2));
  }
};

}  // namespace lsfm
