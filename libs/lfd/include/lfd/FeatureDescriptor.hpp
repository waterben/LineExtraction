//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file FeatureDescriptor.hpp
/// @brief Feature descriptor creation interfaces and implementations.
/// Provides base classes and utilities for creating various types of feature descriptors
/// for use with OpenCV matchers and custom matching algorithms.

#pragma once

#include <lfd/FeatureFilter.hpp>
#include <opencv2/core/core.hpp>
#include <utility/option_manager.hpp>

#include <map>
#include <sstream>
#include <string>

namespace lsfm {
/// @brief Map from string to cv::Mat for storing preprocessed image data
typedef std::map<std::string, cv::Mat> MatMap;


/// @brief Simple feature descriptor creator interface for Mat-based descriptors.
/// Creates matrix-style descriptors where each row represents one descriptor.
/// Suitable for use with OpenCV matchers; row distance computed using cv::norm.
/// @tparam FT Float type for descriptor elements
/// @tparam GT Geometric type (e.g., LineSegment) to create descriptors from
template <class FT, class GT>
class FdcMatI {
 public:
  typedef FT float_type;      ///< Float type alias
  typedef GT geometric_type;  ///< Geometric type alias
  virtual ~FdcMatI() {}

  /// @brief Create descriptors from geometric object vector.
  /// Allows any container type (vector, list, etc.) for geometric objects.
  /// @tparam GV Container template for geometric objects
  /// @tparam Args Additional template arguments
  /// @param input Geometric object container
  /// @param dst Output cv::Mat where rows are descriptors
  template <template <class, class...> class GV, class... Args>
  void createMat(const GV<GT, Args...>& input, cv::Mat& dst) {
    int rows = static_cast<int>(input.size()), cols = static_cast<int>(this->size());

    // if dst is not initialized properly, do it now
    if (dst.empty() || dst.rows != rows || dst.cols != cols) dst.create(rows, cols, cv::DataType<FT>::type);

    int r = 0;
    for_each(input.begin(), input.end(), [this, &r, &dst](const GT& gt) {
      // cv::Mat tmp = dst.row(r++); // fix for gcc
      // this->create(gt, tmp);
      this->create(gt, dst.template ptr<FT>(r++));
    });
  }

  /// @brief Create a single descriptor from a geometric object.
  /// @param input The geometric object
  /// @param dst Output cv::Mat for the descriptor
  virtual void create(const GT& input, cv::Mat& dst) = 0;

  /// @brief Get size of a single descriptor in elements.
  /// @return Number of columns in cv::Mat descriptor
  virtual size_t size() const = 0;

 protected:
  /// @brief Create a single descriptor from a geometric object (pointer version).
  /// @param input The geometric object
  /// @param dst Pointer to output descriptor array
  virtual void create(const GT& input, FT* dst) = 0;
};


/// @brief Custom feature descriptor creator interface with descriptor objects.
/// Supports custom descriptor types that require their own matcher implementations.
/// @tparam FT Float type for distance calculations
/// @tparam GT Geometric type (e.g., LineSegment) to create descriptors from
/// @tparam DT Descriptor type with distance() method and name() method
template <class FT, class GT, class DT>
class FdcObjI {
 public:
  typedef FT float_type;       ///< Float type alias
  typedef GT geometric_type;   ///< Geometric type alias
  typedef DT descriptor_type;  ///< Descriptor type alias
  virtual ~FdcObjI() {}

  /// @brief Create descriptors from geometric object vector.
  /// Allows any container type for geometric objects and descriptors.
  /// @tparam GV Geometric object vector container template
  /// @tparam Args1 Additional template arguments for GV
  /// @tparam DV Descriptor vector container template
  /// @tparam Args2 Additional template arguments for DV
  /// @param input Geometric object container
  /// @param dst Output descriptor container
  template <template <class, class...> class GV, class... Args1, template <class, class...> class DV, class... Args2>
  void createList(const GV<GT, Args1...>& input, DV<DT, Args2...>& dst) {
    dst.resize(input.size());

    auto iter = dst.begin();
    for_each(input.begin(), input.end(), [this, &iter](const GT& gt) { this->create(gt, *(iter++)); });
  }

  /// @brief Create descriptors in geometric object vector.
  /// Updates descriptor field directly in each geometric object.
  /// @tparam GV Geometric vector container template
  /// @tparam Args Additional template arguments for GV
  /// @param input Geometric object container with descriptor fields
  template <template <class, class...> class GV, class... Args1>
  void createList(GV<GT, Args1...>& input) {
    for_each(input.begin(), input.end(), [this](const GT& gt) { this->create(gt, gt.descriptor); });
  }

  /// @brief Create descriptors with mask support.
  /// Uses inverted mask (!=0 not masked, ==0 masked) to create empty descriptors for masked indices.
  /// @tparam GV Geometric vector container template
  /// @tparam Args1 Additional template arguments for GV
  /// @tparam MV Mask vector type
  /// @tparam DV Descriptor vector container template
  /// @tparam Args2 Additional template arguments for DV
  /// @param input Geometric object container
  /// @param mask Inverted mask vector
  /// @param dst Output descriptor container
  template <template <class, class...> class GV,
            class... Args1,
            class MV,
            template <class, class...>
            class DV,
            class... Args2>
  void createList(const GV<GT, Args1...>& input, const MV& mask, DV<DT, Args2...>& dst) {
    dst.resize(input.size());

    auto miter = mask.begin();
    auto diter = dst.begin();
    for (auto in_iter = input.begin(); in_iter != input.end(); ++in_iter, ++miter, ++diter) {
      if (*miter)
        this->create(*in_iter, *diter);
      else
        *diter = DT();
    }
  }

  /// @brief Create descriptors in geometric objects with mask support.
  /// Uses inverted mask to skip descriptor creation for masked features.
  /// @tparam GV Geometric vector container template
  /// @tparam Args1 Additional template arguments for GV
  /// @tparam MV Mask vector type
  /// @param input Geometric object container with descriptor fields
  /// @param mask Inverted mask vector
  template <template <class, class...> class GV, class... Args1, class MV>
  void createList(GV<GT, Args1...>& input, const MV& mask) {
    auto miter = mask.begin();
    for (auto in_iter = input.begin(); in_iter != input.end(); ++in_iter, ++miter) {
      if (*miter)
        this->create(*in_iter, in_iter->descriptor);
      else
        in_iter->descriptor = DT();
    }
  }

  /// @brief Create a single descriptor from a geometric object.
  /// @param input The geometric object
  /// @param dst Output descriptor
  virtual void create(const GT& input, DT& dst) = 0;
};


/// @brief Simple Mat-based feature descriptor creator.
/// Creates descriptors as cv::Mat where each row represents one descriptor.
/// Can be configured via option manager interface.
/// @tparam FT Float type for descriptor elements
/// @tparam GT Geometric type to create descriptors from
template <class FT, class GT>
class FdcMat : public FdcMatI<FT, GT>, public OptionManager {
 public:
  typedef cv::Ptr<FdcMat<FT, GT>> Ptr;  ///< Smart pointer type

  virtual ~FdcMat() {}

  /// @brief Set internal processing data after initialization.
  /// Allows updating image derivatives, gradients, or other preprocessed data.
  /// @param data Map of preprocessed image data (keys: "img", "dx", "dy", etc.)
  virtual void setData(const MatMap& data) = 0;

  using FdcMatI<FT, GT>::create;
};

/// @brief Custom feature descriptor creator with configurable options.
/// Creates custom descriptor types with configurable behavior through option manager.
/// @tparam FT Float type for distance calculations
/// @tparam GT Geometric type to create descriptors from
/// @tparam DT Custom descriptor type
template <class FT, class GT, class DT>
class FdcObj : public FdcObjI<FT, GT, DT>, public OptionManager {
 public:
  typedef FT float_type;                    ///< Float type alias
  typedef GT geometric_type;                ///< Geometric type alias
  typedef DT descriptor_type;               ///< Descriptor type alias
  typedef cv::Ptr<FdcObj<FT, GT, DT>> Ptr;  ///< Smart pointer type

  virtual ~FdcObj() {}

  /// @brief Set internal processing data after initialization.
  /// @param data Map of preprocessed image data
  virtual void setData(const MatMap& data) = 0;

  using FdcObjI<FT, GT, DT>::create;
};

/// @brief Combined descriptor creator supporting both Mat and custom descriptor types.
/// Allows flexibility in descriptor format - can create both Mat-style and custom object descriptors.
/// @tparam FT Float type for distance calculations
/// @tparam GT Geometric type to create descriptors from
/// @tparam DT Custom descriptor type
template <class FT, class GT, class DT>
class Fdc : public FdcObjI<FT, GT, DT>, public FdcMatI<FT, GT>, public OptionManager {
 public:
  typedef FT float_type;                 ///< Float type alias
  typedef GT geometric_type;             ///< Geometric type alias
  typedef DT descriptor_type;            ///< Descriptor type alias
  typedef cv::Ptr<Fdc<FT, GT, DT>> Ptr;  ///< Smart pointer type

  virtual ~Fdc() {}

  /// @brief Set internal processing data after initialization.
  /// @param data Map of preprocessed image data
  virtual void setData(const MatMap& data) = 0;

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, DT>::create;
};


/// @brief Combined Mat-based descriptor creator from multiple creators.
/// Allows combining multiple simple descriptor creators into one larger descriptor.
/// Concatenates outputs from each creator into a single descriptor vector.
/// @tparam FT Float type for descriptor elements
/// @tparam GT Geometric type to create descriptors from
template <class FT, class GT>
class FdcCombinedMat : public FdcMat<FT, GT> {
 public:
  typedef FT float_type;                     ///< Float type alias
  typedef GT geometric_type;                 ///< Geometric type alias
  typedef typename FdcMat<FT, GT>::Ptr Ptr;  ///< Smart pointer type

  virtual ~FdcCombinedMat() {}
  FdcCombinedMat() : size_(0) {}

  /// @brief Add a descriptor creator to combine.
  /// @param creator The descriptor creator to add
  void addCreator(const Ptr& creator) {
    creators_.push_back(creator);
    size_ = 0;
    for_each(creators_.begin(), creators_.end(), [&](const Ptr& desc_creator) { this->size_ += desc_creator->size(); });
  }

  /// @brief Create a descriptor by combining outputs from all creators.
  /// @param input The geometric object
  /// @param dst Output cv::Mat for combined descriptor
  virtual void create(const GT& input, cv::Mat& dst) {
    int cols = static_cast<int>(this->size());
    // if dst is not initialized properly, do it now
    if (dst.empty() || dst.cols != cols) dst.create(1, cols, cv::DataType<FT>::type);

    int off = 0;
    for_each(creators_.begin(), creators_.end(), [&](Ptr creator) {
      cols = creator->size();
      creator->create(input, dst.colRange(off, cols + off));
      off += cols;
    });
  }

  /// @brief Get total size of combined descriptors.
  /// @return Total number of elements across all creators
  virtual size_t size() const { return size_; }

  /// @brief Set internal processing data for all creators.
  /// @param data Map of preprocessed image data
  virtual void setData(const MatMap& data) {
    for_each(creators_.begin(), creators_.end(), [&](Ptr creator) { creator->setData(data); });
  }

  using FdcMatI<FT, GT>::create;

 private:
  std::vector<Ptr> creators_;  ///< Vector of descriptor creators
  size_t size_;                ///< Total descriptor size
};

/// @brief Feature descriptor wrapper for simple Mat-based descriptors.
/// Wraps cv::Mat descriptors with a distance computation method compatible with matching.
/// @tparam FT Float type for descriptor elements and distances
template <class FT>
struct FdMat {
  typedef FT float_type;  ///< Float type alias

  FdMat() {}
  /// @brief Construct from a cv::Mat.
  /// @param d The descriptor matrix
  FdMat(const cv::Mat& d) : data(d) {}

  cv::Mat data;  ///< Descriptor data

  /// @brief Compute L2 distance to another descriptor.
  /// @param rhs The other descriptor
  /// @return L2 norm distance
  FT distance(const FdMat<FT>& rhs) const { return static_cast<FT>(norm(data, rhs.data, cv::NORM_L2)); }

  /// @brief Get a human-readable name for this descriptor type.
  /// @return The string "simple"
  std::string name() const { return "simple"; }

  /// @brief Create descriptors from a Mat of simple features.
  /// Converts rows of a descriptor matrix into individual FdMat objects.
  /// @tparam DV Descriptor vector container template
  /// @tparam DVargs Additional template arguments for DV
  /// @param sd Source descriptor matrix (each row is a descriptor)
  /// @param d Output descriptor vector
  template <template <class, class...> class DV, class... DVargs>
  static void create(const cv::Mat& sd, DV<FdMat<FT>, DVargs...>& d) {
    d.clear();
    d.resize(sd.rows);
    for (int i = 0; i != sd.rows; ++i) {
      d[i].data = sd.row(i);
    }
  }
};


/// @brief Dynamic descriptor interface with polymorphic distance computation.
/// Base class for type-erased descriptors using polymorphism.
/// Allows different descriptor types to be stored in the same container.
/// @tparam FT Float type for distance calculations
template <class FT>
struct FdDynamicI {
  typedef FT float_type;  ///< Float type alias
  virtual ~FdDynamicI() {}

  /// @brief Compute distance to another descriptor.
  /// Supports any metric, not necessarily L2.
  /// @param rhs The other descriptor
  /// @return Distance value
  virtual FT distance(const FdDynamicI<FT>& rhs) const = 0;

  /// @brief Get unique name for this descriptor type.
  /// Should be unique to enable type checking.
  /// @return Descriptor type name
  virtual std::string name() const = 0;
};

/// @brief Type-erased wrapper for dynamic descriptors.
/// Adapts any descriptor type to the FdDynamicI interface.
/// @tparam DT Descriptor type with distance() and name() methods
template <class DT>
struct FdDynamic : public FdDynamicI<typename DT::float_type> {
  typedef typename DT::float_type float_type;  ///< Float type alias

  virtual ~FdDynamic() {}

  DT data;  ///< The wrapped descriptor

  /// @brief Compute distance to another dynamic descriptor.
  /// @param rhs The other descriptor (must be same type)
  /// @return Distance value
  /// @throws std::runtime_error if descriptor types don't match
  virtual float_type distance(const FdDynamicI<float_type>& rhs) const {
    if (rhs.name() != this->name()) throw std::runtime_error("Invalid descriptor type!");
    return data.distance(reinterpret_cast<const FdDynamic<DT>*>(&rhs)->data);
  }

  /// @brief Get unique name for this descriptor type.
  /// @return Prefixed descriptor name ("dynamic_" + data.name())
  virtual std::string name() const { return "dynamic_" + data.name(); }
};


/// @brief Compute distance between two descriptors.
/// Dispatches to the distance() method of the descriptor.
/// @tparam DT Descriptor type
/// @param lhs First descriptor
/// @param rhs Second descriptor
/// @return Distance value
template <class DT>
inline typename DT::float_type fdDistance(const DT& lhs, const DT& rhs) {
  return lhs.distance(rhs);
}

/// @brief Compute distance between two dynamic descriptors.
/// Dispatches to polymorphic distance() method.
/// @tparam FT Float type
/// @param lhs First descriptor
/// @param rhs Second descriptor
/// @return Distance value
template <class FT>
inline FT fdDistance(const FdDynamicI<FT>& lhs, const FdDynamicI<FT>& rhs) {
  return lhs.distance(rhs);
}

// with the dynamic FD you can hold different descriptors in the same container.
// to create a lot of single descriptors in heap is expensive, so only use
// it to reference different sets of descriptors in one container.
// To combine feature descriptors create a new descriptor:
/*template<class FT>
class MyCombinedFD {
    FD1 fd1;
    FD2 fd2;
    FD3 fd3;

    FT distance(const MyCombinedFD& rhs) const {
        FT ret = 0, tmp = fd1.distance(rhs.fd1);
        tmp *= tmp;
        ret += tmp;
        rmp = fd2.distance(rhs.fd2);
        tmp *= tmp;
        ret += tmp;
        rmp = fd3.distance(rhs.fd3);
        tmp *= tmp;
        ret += tmp;
        return sqrt(ret);
    }

    std::string name() const {
        return fd1.name() + fd2.name() + fd3.name();
    }

};*/
// use single creator of creator class to create descriptors or add internal static
// creator helper


}  // namespace lsfm
