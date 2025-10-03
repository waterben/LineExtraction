/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


#ifndef _LFD_FEATUREDESCRIPTOR_HPP_
#define _LFD_FEATUREDESCRIPTOR_HPP_
#ifdef __cplusplus

#  include <lfd/FeatureFilter.hpp>
#  include <opencv2/core/core.hpp>
#  include <utility/option_manager.hpp>

#  include <map>
#  include <sstream>
#  include <string>

namespace lsfm {
typedef std::map<std::string, cv::Mat> MatMap;


//! Simple feature descriptor creator interface -> this will create mat
//! style descriptors that can be used with opencv matchers (each row
//! in mat represents one descriptor).
//! use norm for a row in mat to compute the distance between two descriptors
template <class FT, class GT>
class FdcMatI {
 public:
  typedef FT float_type;
  typedef GT geometric_type;
  virtual ~FdcMatI() {}

  //! create simple descriptors from geometric object vector and write to mat row by row
  //! This will allow to pass all kind of container types for GT (vector, list, etc)
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

  //! create single simple descriptor from geometric object
  virtual void create(const GT& input, cv::Mat& dst) = 0;

  //! get size of single descriptor (cols in cv::Mat)
  virtual size_t size() const = 0;

 protected:
  //! create single simple descriptor from geometric object
  virtual void create(const GT& input, FT* dst) = 0;
};

//! Custom freature descriptor creator interface for custom descriptor types (requires custom matcher)
template <class FT, class GT, class DT>
class FdcObjI {
 public:
  typedef FT float_type;
  typedef GT geometric_type;
  typedef DT descriptor_type;
  virtual ~FdcObjI() {}

  //! create descriptors from geometric object vector and write to descriptor vector
  //! This will allow to pass all kind of container types for GT and DT (vector, list, etc)
  template <template <class, class...> class GV, class... Args1, template <class, class...> class DV, class... Args2>
  void createList(const GV<GT, Args1...>& input, DV<DT, Args2...>& dst) {
    dst.resize(input.size());

    auto iter = dst.begin();
    for_each(input.begin(), input.end(), [this, &iter](const GT& gt) { this->create(gt, *(iter++)); });
  }

  //! create descriptors from geometric object vector and write to descriptor in geometric object vector
  //! this only works for geometric types that include the descriptor
  template <template <class, class...> class GV, class... Args1>
  void createList(GV<GT, Args1...>& input) {
    for_each(input.begin(), input.end(), [this](const GT& gt) { this->create(gt, gt.descriptor); });
  }

  //! create descriptors from geometric object vector and write to descriptor vector
  //! This will allow to pass all kind of container types for GT and DT (vector, list, etc)
  //! uses inverted mask to create empty descriptors for masked indexes (index exist, but empty, inverted masks, != is
  //! not masked, == 0 is masked)
  template <template <class, class...> class GV,
            class... Args1,
            class MV,
            template <class, class...>
            class DV,
            class... Args2>
  void createList(const GV<GT, Args1...>& input, const MV& mask, DV<DT, Args2...>& dst) {
    dst.resize(input.size());

    auto miter = mask.begin();
    auto iter = dst.begin();
    for_each(input.begin(), input.end(), [this, &iter, &miter](const GT& gt) {
      if (*miter++)
        this->create(gt, *iter++);
      else
        *iter++ = DT();
    });
  }

  //! create descriptors from geometric object vector and write to descriptor in geometric object vector
  //! uses inverted mask to create empty descriptors for masked indexes (index exist, but empty, inverted masks, != is
  //! not masked, == 0 is masked)
  template <template <class, class...> class GV, class... Args1, class MV>
  void createList(GV<GT, Args1...>& input, const MV& mask) {
    auto miter = mask.begin();
    for_each(input.begin(), input.end(), [this, &miter](GT& gt) {
      if (*miter++)
        this->create(gt, gt.descriptor);
      else
        gt.descriptor = DT();
    });
  }

  //! create single descriptor from single geometric object
  virtual void create(const GT& input, DT& dst) = 0;
};

//! Simple feature descriptor creator -> this will create mat style descriptors that
//! can be used with opencv matchers (each row in mat represents one
//! descriptor).
//! use norm for a row in mat to compute the distance between two descriptors
template <class FT, class GT>
class FdcMat : public FdcMatI<FT, GT>, public OptionManager {
 public:
  typedef cv::Ptr<FdcMat<FT, GT>> Ptr;

  virtual ~FdcMat() {}

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) = 0;

  using FdcMatI<FT, GT>::create;
};

//! Custom feature descriptor creator
template <class FT, class GT, class DT>
class FdcObj : public FdcObjI<FT, GT, DT>, public OptionManager {
 public:
  typedef FT float_type;
  typedef GT geometric_type;
  typedef DT descriptor_type;
  typedef cv::Ptr<FdcObj<FT, GT, DT>> Ptr;

  virtual ~FdcObj() {}

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) = 0;

  using FdcObjI<FT, GT, DT>::create;
};

//! Feature descriptor creator (combines simple + custom)
template <class FT, class GT, class DT>
class Fdc : public FdcObjI<FT, GT, DT>, public FdcMatI<FT, GT>, public OptionManager {
 public:
  typedef FT float_type;
  typedef GT geometric_type;
  typedef DT descriptor_type;
  typedef cv::Ptr<Fdc<FT, GT, DT>> Ptr;

  virtual ~Fdc() {}

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) = 0;

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, DT>::create;
};


//! allows to combine multiple simple feature descriptor creators
//! to one simple descritpor
template <class FT, class GT>
class FdcCombinedMat : public FdcMat<FT, GT> {
 public:
  typedef FT float_type;
  typedef GT geometric_type;
  typedef typename FdcMat<FT, GT>::Ptr Ptr;

  virtual ~FdcCombinedMat() {}
  FdcCombinedMat() : size_(0) {}

  void addCreator(const Ptr& creator) {
    creators_.push_back(creator);
    size_ = 0;
    for_each(creators_.begin(), creators_.end(), [&](const Ptr& creator) { this->size_ += creator->size(); });
  }

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

  virtual size_t size() const { return size_; }

  virtual void setData(const MatMap& data) {
    for_each(creators_.begin(), creators_.end(), [&](Ptr creator) { creator->setData(data); });
  }

  using FdcMatI<FT, GT>::create;

 private:
  std::vector<Ptr> creators_;
  size_t size_;
};

//! feature descriptor from simple feature descriptor (convert Mat to Obj with distance method)
template <class FT>
struct FdMat {
  typedef FT float_type;

  FdMat() {}
  FdMat(const cv::Mat& d) : data(d) {}

  cv::Mat data;

  FT distance(const FdMat<FT>& rhs) const { return static_cast<FT>(norm(data, rhs.data, cv::NORM_L2)); }

  std::string name() const {
    // std::ostringstream os;
    // os << ID;
    // return "simple_" + os.str();
    return "simple";
  }

  //! create feature descriptor from simple feature mat
  template <template <class, class...> class DV, class... DVargs>
  static void create(const cv::Mat& sd, DV<FdMat<FT>, DVargs...>& d) {
    d.clear();
    d.resize(sd.rows);
    for (int i = 0; i != sd.rows; ++i) {
      d[i].data = sd.row(i);
    }
  }
};


//! Dynamic feature descriptor interface (uses polymorphism)
template <class FT>
struct FdDynamicI {
  typedef FT float_type;
  virtual ~FdDynamicI() {}

  //! get distance between this an other descriptor -> can be any metric
  virtual FT distance(const FdDynamicI<FT>& rhs) const = 0;

  //! get name of descritpor (should be unique)
  virtual std::string name() const = 0;
};

//! create dynamic descriptor from non dynamic descriptor type
template <class DT>
struct FdDynamic : public FdDynamicI<typename DT::float_type> {
  typedef typename DT::float_type float_type;

  virtual ~FdDynamic() {}

  DT data;
  //! get distance between this an other descriptor -> can be any metric
  virtual float_type distance(const FdDynamicI<float_type>& rhs) const {
    if (rhs.name() != this->name()) throw std::runtime_error("Invalid descriptor type!");
    // since we know the type, just do a faster reinterpret cast
    return data.distance(reinterpret_cast<const FdDynamic<DT>*>(rhs)->data);
  }

  virtual std::string name() const { return "dynamic_" + data.name(); }
};


template <class DT>
inline typename DT::float_type fdDistance(const DT& lhs, const DT& rhs) {
  return lhs.distance(rhs);
}

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
#endif
#endif
