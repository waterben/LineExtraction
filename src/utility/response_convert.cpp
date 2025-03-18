
#include <utility/response_convert.hpp>

namespace lsfm {

cv::Mat convertMag(const cv::Mat& src) {
  double vmin, vmax;
  cv::minMaxIdx(src, &vmin, &vmax);
  return convertMag(src, vmin, vmax);
}

cv::Mat convertMag(const cv::Mat& src, const Range<double>& range) { return convertMag(src, range.lower, range.upper); }

cv::Mat convertMag(const cv::Mat& src, double vmin, double vmax) {
  cv::Mat mag = src.clone();
  mag.convertTo(mag, CV_32F);
  mag /= vmax;
  mag *= 255;
  mag.convertTo(mag, CV_8U);
  return mag;
}

cv::Mat convertLaplace(const cv::Mat& src) {
  cv::Mat lap = cv::abs(src);
  return convertMag(lap);
}

cv::Mat convertLaplace(const cv::Mat& src, const Range<double>& range) {
  return convertLaplace(src, range.lower, range.upper);
}

cv::Mat convertLaplace(const cv::Mat& src, double vmin, double vmax) {
  cv::Mat lap = cv::abs(src);
  return convertMag(lap, 0, std::max(std::abs(vmin), std::abs(vmax)));
}

}  // namespace lsfm
