#ifndef STEREO_MATCHER_H
#define STEREO_MATCHER_H

#include <vector>
#include <stereo/stereo_correlation.hpp>

std::vector<lsfm::StereoCorrelation> getMatchingLines(Mat im_grayLeft, Mat im_grayRight);
std::vector<lsfm::StereoCorrelation> getMatchingLines(Mat im_grayLeft, Mat im_grayRight, cv::Mat &imLeftMod, cv::Mat &imRightMod);


#endif // STEREO_MATCHER_H
