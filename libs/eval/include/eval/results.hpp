//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file results.hpp
/// @brief Evaluation results data structures.

#pragma once

#include <edge/zc.hpp>
#include <imgproc/filter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>


namespace lsfm {

/// @brief Convert dir value to color vector
template <class FT>
inline cv::Vec3b dirColor(FT val) {
  static FT r = static_cast<FT>(CV_PI / 4);
  static cv::Vec3b colors[] = {cv::Vec3b(255, 0, 150), cv::Vec3b(255, 0, 255), cv::Vec3b(0, 0, 255),
                               cv::Vec3b(0, 150, 255), cv::Vec3b(0, 255, 255), cv::Vec3b(0, 255, 0),
                               cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 0),   cv::Vec3b(255, 0, 150)};
  FT div = val / r;
  int divi = static_cast<int>(div);
  cv::Vec<FT, 3> a = colors[divi + 4], b = colors[divi + 4 + (val < 0 ? -1 : 1)];
  return a + (b - a) * (std::abs(div) - std::abs(divi));
}

/// @brief Convert mat to mat with FT type
template <class FT>
inline cv::Mat convertTo(const cv::Mat& in) {
  cv::Mat out;
  if (in.type() != cv::DataType<FT>::type)
    in.convertTo(out, cv::DataType<FT>::type);
  else
    in.copyTo(out);
  return out;
}

/// @brief Apply border to input mat without copy
cv::Mat applyBorder(cv::Mat& inout,
                    int border,
                    int borderType = cv::BORDER_CONSTANT,
                    const cv::Scalar& color = cv::Scalar());

/// @brief Apply border to input mat and return new mat
cv::Mat applyBorderCopy(const cv::Mat& in,
                        int border,
                        int borderType = cv::BORDER_CONSTANT,
                        const cv::Scalar& color = cv::Scalar());

/// @brief Create non-maxima suppression image from edge map
cv::Mat createNMS(const cv::Mat& emap);

/// @brief Save direction map
template <class FT>
void saveDir(const cv::Mat& data, const std::string& name, const cv::Mat& mask = cv::Mat(), int border = 0) {
  cv::Mat tmp(data.size(), CV_8UC3);
  cv::Vec3b* pdst = tmp.ptr<cv::Vec3b>();
  if (data.isContinuous() && data.isContinuous()) {
    const FT* pd = data.ptr<FT>();
    int size = data.rows * data.cols;
    for (int j = 0; j != size; ++j) {
      pdst[j] = dirColor(pd[j]);
    }
  } else {
    for (int i = 0; i != data.rows; ++i, pdst += data.cols) {
      const FT* pd = data.ptr<FT>(i);
      for (int j = 0; j != data.cols; ++j) {
        pdst[j] = dirColor(pd[j]);
      }
    }
  }
  if (!mask.empty()) tmp.setTo(cv::Scalar(0, 0, 0), mask == 0);
  cv::imwrite(name + ".png", applyBorder(tmp, border));
}

/// @brief Save phase map
template <class FT>
void savePhase(const cv::Mat& data, const std::string& name, const cv::Mat& energy, int border = 0) {
  double vmin, vmax;
  cv::minMaxIdx(energy, &vmin, &vmax);
  cv::Mat en = energy / vmax;

  cv::Mat tmp(data.size(), CV_8UC3);
  cv::Vec3b* pdst = tmp.ptr<cv::Vec3b>();
  if (data.isContinuous() && en.isContinuous()) {
    const FT* pd = data.ptr<FT>();
    const FT* pe = en.ptr<FT>();
    int size = data.rows * data.cols;
    for (int j = 0; j != size; ++j) {
      pdst[j] = dirColor(pd[j]) * pe[j];
    }
  } else {
    for (int i = 0; i != data.rows; ++i, pdst += data.cols) {
      const FT* pd = data.ptr<FT>(i);
      const FT* pe = en.ptr<FT>(i);
      for (int j = 0; j != data.cols; ++j) {
        pdst[j] = dirColor(pd[j]) * pe[j];
      }
    }
  }
  cv::imwrite(name + ".png", applyBorder(tmp, border));
}

/// @brief Save edge map
void saveEdge(const cv::Mat& data, const std::string& name, int border = 0);

/// @brief Save normalized data
void saveNormalized(const cv::Mat& data, const std::string& name, int border = 0);

/// @brief Save filter results
template <class FT>
void saveFilterResults(const lsfm::FilterResults& results,
                       const std::string& name,
                       bool sqr = false,
                       int border = 0,
                       double th_low = 0,
                       double th_high = 0) {
  static ZeroCrossing<FT, FT, FT, FastZC<FT, FT, FT>> zc;
  static NonMaximaSuppression<FT, FT, FT, FastNMS8<FT, FT, FT>> nms;
  lsfm::FilterResults::const_iterator f;
  double low, high;

  for_each(results.begin(), results.end(), [&](const FilterResult& result) {
    cv::Mat tmp, tmpx, tmpy;
    std::string filter = result.first;
    tmp = convertTo<FT>(result.second.data);
    if (filter == "phase") {
      f = results.find("energy");
      if (f != results.end())
        savePhase<FT>(tmp, name + "_" + f->first, f->second.data, border);
      else
        saveDir<FT>(tmp, name + "_" + f->first, border);
      return;
    }

    if (filter == "dir") {
      f = results.find("odd");
      if (f == results.end()) f = results.find("mag");
      if (f != results.end()) {
        cv::Mat mask = convertTo<FT>(f->second.data);
        if (sqr) {
          cv::sqrt(mask, mask);
        }
        saveDir<FT>(tmp, name + "_" + f->first, mask);
      } else
        saveDir<FT>(tmp, name + "_" + f->first);
      return;
    }

    if (filter == "laplace") {
      if (th_high) {
        zc.process(tmpx, f->second.range.size() * th_low, f->second.range.size() * th_high);
        saveEdge(zc.hysteresis(), name + "_edge_laplace", border);
      }
    }

    else if (filter == "mag") {
      // std::cout << "mag range upper: " << f->second.range.upper << std::endl;
      low = f->second.range.upper * th_low;
      high = f->second.range.upper * th_high;
      f = results.find("gx");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpx = convertTo<FT>(f->second.data);

      f = results.find("gy");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpy = convertTo<FT>(f->second.data);

      if (th_high) {
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_edge_mag", border);
      }
    }

    else if (filter == "even") {
      // std::cout << "even range upper: " << f->second.range.upper << std::endl;

      low = f->second.range.size() * th_low;
      high = f->second.range.size() * th_high;
      f = results.find("oddx");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpx = convertTo<FT>(f->second.data);

      f = results.find("oddy");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpy = convertTo<FT>(f->second.data);

      if (th_high && !tmpx.empty() && !tmpy.empty()) {
        zc.process(tmpx, tmpy, tmp, low, high);
        saveEdge(zc.hysteresis(), name + "_edge_even", border);
      }
      tmpx = cv::Mat();
      tmpy = cv::Mat();
    }

    else if (filter == "odd") {
      // std::cout << "odd range upper: " << f->second.range.upper << std::endl;

      low = f->second.range.upper * th_low;
      high = f->second.range.upper * th_high;

      f = results.find("oddx");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpx = convertTo<FT>(f->second.data);

      f = results.find("oddy");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpy = convertTo<FT>(f->second.data);

      if (th_high && !tmpx.empty() && !tmpy.empty()) {
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_edge_odd", border);
      }
    }

    else if (filter == "energy" || filter == "pc") {
      // std::cout << filter << " range upper: " << f->second.range.upper << std::endl;

      low = f->second.range.upper * th_low;
      high = f->second.range.upper * th_high;

      f = results.find("oddx");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpx = convertTo<FT>(f->second.data);

      f = results.find("oddy");
      if (f->second.data.type() != cv::DataType<FT>::type) tmpy = convertTo<FT>(f->second.data);

      if (th_high && !tmpx.empty() && !tmpy.empty()) {
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_edge_" + filter, border);
      }

      tmpx = cv::Mat();
      tmpy = cv::Mat();
    }

    else if (filter == "pclx" || filter == "pcly") {
      // std::cout << "pcl range upper: " << f->second.range.upper << std::endl;

      if (th_high) {
        zc.process(tmp, f->second.range.size() * th_low * th_low, f->second.range.size() * th_high * th_high);
        saveEdge(zc.hysteresis(), name + "_edge_" + filter, border);
      }

    } else
      // nothing to save
      return;

    if (sqr) cv::sqrt(tmp, tmp);

    saveNormalized(tmp, name + "_" + filter, border);
    if (!tmpx.empty()) saveNormalized(tmpx, name + "_" + filter + "_x", border);
    if (!tmpy.empty()) saveNormalized(tmpy, name + "_" + filter + "_y", border);
  });
}
}  // namespace lsfm
