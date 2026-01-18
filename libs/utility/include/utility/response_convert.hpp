#pragma once


#include <opencv2/imgproc.hpp>
#include <utility/range.hpp>


namespace lsfm {

cv::Mat convertMag(const cv::Mat& src);
cv::Mat convertMag(const cv::Mat& src, const Range<double>& range);
cv::Mat convertMag(const cv::Mat& src, double vmin, double vmax);

cv::Mat convertLaplace(const cv::Mat& src);
cv::Mat convertLaplace(const cv::Mat& src, const Range<double>& range);
cv::Mat convertLaplace(const cv::Mat& src, double vmin, double vmax);

}  // namespace lsfm
