//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file ld_base.hpp
/// @brief Base class for line detectors.
/// This template class defines the interface for all line detection algorithms.

#pragma once

#include <geometry/line.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <utility/value_manager.hpp>

namespace lsfm {

/// @brief Descriptor entry for image data layers.
/// This structure associates a name and description with each layer of processed image data,
/// such as gradient magnitude, direction map, or edge map. Used to document the output of
/// line detection algorithms.
struct DataDescriptorEntry {
  /// @brief Create a descriptor entry.
  /// @param n Name of the data layer (e.g., "gradient_magnitude")
  /// @param d Human-readable description of the data layer
  DataDescriptorEntry(const std::string& n = std::string(), const std::string d = std::string())
      : name(n), description(d) {}

  std::string name{};         ///< Name of the data layer
  std::string description{};  ///< Description of the data layer
};

/// @brief Collection of image data layer descriptors.
typedef std::vector<DataDescriptorEntry> DataDescriptor;

/// @brief Abstract base class for line detectors.
/// This template class defines the interface for all line detection algorithms.
/// Derived classes implement various line segment detection methods.
/// @tparam FT Floating-point type for line coordinates (e.g., float, double)
/// @tparam LPT Line point template class (default Vec2)
template <class FT, template <class> class LPT = Vec2>
class LdBase : public ValueManager {
 public:
  typedef FT float_type;       ///< Floating-point type for coordinates
  typedef LPT<FT> line_point;  ///< Line point type
  template <class A>
  using line_point_template = LPT<A>;  ///< Line point template alias

  typedef lsfm::Line<FT, LPT> Line;        ///< Line type
  typedef std::vector<Line> LineVector;    ///< Vector of detected lines
  typedef std::vector<cv::Mat> ImageData;  ///< Additional image data layers (gradients, edges, etc.)

  /// @brief Virtual destructor for proper cleanup of derived classes.
  virtual ~LdBase() {}

  /// @brief Detect lines in the input image.
  /// This is the main detection method that must be implemented by derived classes.
  /// @param image Input image. Typically 8-bit single-channel grayscale.
  virtual void detect(const cv::Mat& image) = 0;

  /// @brief Convenience method: Detect lines and retrieve results.
  /// @param image Input image
  /// @param l Output vector of detected lines
  inline void detect(const cv::Mat& image, LineVector& l) {
    detect(image);
    l = lines();
  }

  /// @brief Convenience method: Detect lines and retrieve both lines and auxiliary data.
  /// @param image Input image
  /// @param l Output vector of detected lines
  /// @param id Output vector of auxiliary image data (gradients, edges, etc.)
  inline void detect(const cv::Mat& image, LineVector& l, ImageData& id) {
    detect(image);
    l = lines();
    id = imageData();
  }

  /// @brief Get the detected line vector.
  /// @return Const reference to the vector of detected lines
  virtual const LineVector& lines() const { return lines_; }

  /// @brief Get descriptors for all auxiliary image data layers.
  /// Each entry describes one layer of image data (e.g., "gradient_magnitude", "edge_map").
  /// Use this to understand what data is available via imageData().
  /// @return Image data descriptor containing name and description for each layer
  virtual const DataDescriptor& imageDataDescriptor() const {
    static DataDescriptor id;
    return id;
  }

  /// @brief Get auxiliary image data from the last detection.
  /// Returns processed image data such as gradients, directions, and edge maps
  /// that were computed during detection. Use imageDataDescriptor() to understand the
  /// content and order of layers.
  /// @return Vector of cv::Mat, one for each data layer. Empty if no auxiliary data is provided.
  /// @code{cpp}
  /// detector->detect(image);
  /// const auto& data = detector->imageData();
  /// const auto& desc = detector->imageDataDescriptor();
  /// for (size_t i = 0; i < data.size(); ++i) {
  ///   cv::Mat layer = data[i];
  ///   std::string name = desc[i].name;
  ///   std::string info = desc[i].description;
  /// }
  /// @endcode
  virtual const ImageData& imageData() const {
    static ImageData id;
    return id;
  }

  /// @brief Get a single auxiliary image data layer by name.
  /// Searches for a data layer with the given name and returns it, or an empty Mat if not found.
  /// @param name Name of the data layer (e.g., "gradient_magnitude")
  /// @return The requested image data as cv::Mat, or empty Mat if not found
  const cv::Mat imageData(const std::string& name) const {
    const DataDescriptor& data = imageDataDescriptor();
    typename DataDescriptor::const_iterator f =
        find_if(data.begin(), data.end(), [&name](const DataDescriptorEntry& e) { return (e.name == name); });
    return f != data.end() ? imageData()[static_cast<size_t>(f - data.begin())] : cv::Mat();
  }

 protected:
  /// @brief Protected constructor for derived classes.
  LdBase() {}

  mutable LineVector lines_{};  ///< Storage for detected lines

  /// @brief Clear all internal detection data.
  /// Called before each detection to reset state. Derived classes should override
  /// to clear their specific data members.
  virtual void clearData() { lines_.clear(); }
};

}  // namespace lsfm
