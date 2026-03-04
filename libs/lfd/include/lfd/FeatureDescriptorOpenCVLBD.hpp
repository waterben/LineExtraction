//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file FeatureDescriptorOpenCVLBD.hpp
/// @brief OpenCV Binary LBD descriptor wrapper for the LFD library.
/// Wraps OpenCV's cv::line_descriptor::BinaryDescriptor to compute 256-bit
/// binary LBD descriptors from line segments detected by our own LSD detector.
/// Uses Hamming distance for matching.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <lfd/lineMatching.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include <limits>
#include <string>
#include <vector>

namespace lsfm {

/// @brief OpenCV Binary LBD descriptor structure.
/// Stores a 256-bit (32 byte) binary descriptor from OpenCV's BinaryDescriptor.
/// Uses Hamming distance for matching instead of L2.
/// @tparam FT Float type for distance return values
template <class FT>
struct FdOpenCVLBD {
  FdOpenCVLBD() {}

  /// @brief Construct from a descriptor matrix row.
  /// @param d The descriptor data (1 x 32, CV_8U)
  explicit FdOpenCVLBD(const cv::Mat& d) : data(d) {}

  cv::Mat data{};  ///< Binary descriptor data (1 x 32, CV_8U = 256 bits)

  /// @brief Compute Hamming distance to another binary descriptor.
  /// @param rhs The other descriptor
  /// @return Hamming distance as FT (returns max float if either descriptor is empty)
  inline FT distance(const FdOpenCVLBD<FT>& rhs) const {
    if (data.empty() || rhs.data.empty()) return std::numeric_limits<FT>::max();
    return static_cast<FT>(cv::norm(data, rhs.data, cv::NORM_HAMMING));
  }

  /// @brief Compute Hamming distance between two descriptors (static version).
  /// @param lhs First descriptor
  /// @param rhs Second descriptor
  /// @return Hamming distance as FT
  static inline FT distance(const FdOpenCVLBD<FT>& lhs, const FdOpenCVLBD<FT>& rhs) { return lhs.distance(rhs); }

  /// @brief Get descriptor type name.
  /// @return String "OpenCVLBD"
  std::string name() const { return "OpenCVLBD"; }
};


/// @brief Feature descriptor creator for OpenCV's Binary LBD.
/// Wraps cv::line_descriptor::BinaryDescriptor to compute 256-bit binary
/// descriptors for line segments. Accepts line segments from any detector
/// (e.g., our LSD implementation) and converts them to OpenCV KeyLine format
/// internally.
///
/// @note OpenCV's BinaryDescriptor may drop keylines that are too short or
///       too close to image borders. Dropped lines receive empty descriptors.
///
/// @tparam FT Float type for computations
/// @tparam GT Geometric type representing line segments
template <class FT, class GT = LineSegment<FT>>
class FdcOpenCVLBD : public FdcObj<FT, GT, FdOpenCVLBD<FT>> {
  cv::Mat image_{};                                      ///< Grayscale input image
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd_{};  ///< OpenCV BinaryDescriptor instance
  int max_dim_ = 0;                                      ///< Max image dimension for KeyLine conversion

  /// @brief Initialize the BinaryDescriptor and update cached image dimension.
  void init() {
    if (!bd_) {
      bd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    }
    max_dim_ = std::max(image_.cols, image_.rows);
  }

 public:
  typedef FdOpenCVLBD<FT> descriptor_type;  ///< The descriptor type produced by this creator

  /// @brief Construct from a grayscale image.
  /// @param img Grayscale image (CV_8U)
  explicit FdcOpenCVLBD(const cv::Mat& img) : image_(img) { init(); }

  /// @brief Construct from a named matrix map.
  /// Looks for "img" or "image" keys in the map.
  /// @param data Map of named matrices
  explicit FdcOpenCVLBD(const MatMap& data) {
    init();
    setData(data);
  }

  /// @brief Create a single binary LBD descriptor for one line segment.
  /// @note This is less efficient than createAll() since BinaryDescriptor
  ///       has per-call overhead. Prefer createAll() for batch processing.
  /// @param line The line segment to describe
  /// @param dst Output descriptor
  void create(const GT& line, FdOpenCVLBD<FT>& dst) override {
    std::vector<cv::line_descriptor::KeyLine> kls = {lineSegment2KeyLine(line, max_dim_, 0)};
    cv::Mat desc;
    bd_->compute(image_, kls, desc);
    if (!desc.empty()) {
      dst.data = desc.row(0).clone();
    } else {
      dst = FdOpenCVLBD<FT>();
    }
  }

  /// @brief Batch-create binary LBD descriptors for all line segments.
  /// This is much more efficient than calling create() in a loop since
  /// BinaryDescriptor::compute() processes all keylines at once.
  /// @note OpenCV may drop some keylines (too short, near borders).
  ///       Dropped segments receive empty descriptors (distance → max).
  /// @tparam GV Container template for geometric objects
  /// @tparam Args Additional template arguments
  /// @param lines Input line segments
  /// @param dst Output descriptors (resized to match lines.size())
  template <template <class, class...> class GV, class... Args>
  void createAll(const GV<GT, Args...>& lines, std::vector<FdOpenCVLBD<FT>>& dst) {
    dst.resize(lines.size());

    // Convert all line segments to OpenCV KeyLine format
    std::vector<cv::line_descriptor::KeyLine> kls;
    kls.reserve(lines.size());
    int idx = 0;
    for (auto it = lines.begin(); it != lines.end(); ++it, ++idx) {
      kls.push_back(lineSegment2KeyLine(*it, max_dim_, idx));
    }

    cv::Mat desc;
    bd_->compute(image_, kls, desc);

    // BinaryDescriptor::compute() may prune keylines. Each remaining
    // keyline retains its class_id (set to original index above).
    for (size_t i = 0; i < kls.size(); ++i) {
      int orig_idx = kls[i].class_id;
      if (orig_idx >= 0 && orig_idx < static_cast<int>(lines.size()) && static_cast<int>(i) < desc.rows) {
        dst[orig_idx].data = desc.row(static_cast<int>(i)).clone();
      }
    }
  }

  /// @brief Get the size of a single descriptor in bytes.
  /// @return 32 (256 bits / 8)
  size_t size() const { return 32; }

  /// @brief Set internal image data after construction.
  /// Accepts "img" or "image" keys from the matrix map.
  /// @param data Map of named matrices
  void setData(const MatMap& data) override {
    auto f = data.find("img");
    if (f != data.end()) {
      image_ = f->second;
    } else {
      f = data.find("image");
      if (f != data.end()) {
        image_ = f->second;
      }
    }
    if (!image_.empty()) {
      max_dim_ = std::max(image_.cols, image_.rows);
    }
  }
};

}  // namespace lsfm
