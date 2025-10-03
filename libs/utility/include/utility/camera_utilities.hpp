#ifndef _CAMERA_UTILITIES_HPP_
#define _CAMERA_UTILITIES_HPP_
#ifdef __cplusplus

#  include <opencv/cv.h>
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/opencv.hpp>

#  include <iostream>

namespace lsfm {


//! Calculates the width of the black border on each side of the image, order: top, bottom, left, right
//! Function has to be changed if any of the Borders reach more than one quater of the width/height into the image
template <class MTYPE>
void findBlackCroppingArea(const cv::Mat img, std::vector<int>& border) {
  cv::Mat upper =
      img(cv::Rect(img.size().width / MTYPE(4), MTYPE(0), img.size().width / MTYPE(2), img.size().height / MTYPE(2)));
  cv::Mat lower =
      img(cv::Rect(img.size().width / 4, img.size().height / 2, img.size().width / 2, img.size().height / 2));
  cv::Mat left = img(cv::Rect(0, img.size().height / 4, img.size().width / 2, img.size().height / 2));
  cv::Mat right =
      img(cv::Rect(img.size().width / 2, img.size().height / 4, img.size().width / 2, img.size().height / 2));

  bool done = false;
  for (int i = upper.rows - 1; i >= 0 && !done; i--) {
    for (int j = 0; j < upper.cols && !done; j++) {
      if (255 != upper.at<MTYPE>(i, j)) {
        border.push_back(i);
        done = true;
      }
    }
  }
  done = false;
  for (int i = 0; i < lower.rows && !done; i++) {
    for (int j = 0; j < lower.cols && !done; j++) {
      if (255 != lower.at<MTYPE>(i, j)) {
        border.push_back(lower.rows - i);
        done = true;
      }
    }
  }
  done = false;
  for (int i = left.cols - 1; i >= 0 && !done; i--) {
    for (int j = 0; j < left.rows && !done; j++) {
      if (255 != left.at<MTYPE>(j, i)) {
        border.push_back(i);
        done = true;
      }
    }
  }
  done = false;
  for (int i = 0; i < right.cols && !done; i++) {
    for (int j = 0; j < right.rows && !done; j++) {
      if (255 != right.at<MTYPE>(j, i)) {
        border.push_back(right.cols - i);
        done = true;
      }
    }
  }
}

//! Calculates Undistort-Rectify Map
int stereoRectification(const std::string intrinsic_filename,
                        const std::string extrinsic_filename,
                        const cv::Size img_size,
                        cv::Mat& P1,
                        cv::Mat& P2,
                        cv::Mat& map11,
                        cv::Mat& map12,
                        cv::Mat& map21,
                        cv::Mat& map22,
                        const double scaleFactor = 1.0) {
  std::cout << "DEBUG: size: " << img_size << std::endl;

  cv::Mat Q;
  cv::Mat R, T, R1, R2;
  cv::Rect roi1, roi2;

  // reading intrinsic parameters
  cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    printf("Failed to open file intrinsic_filename\n");
    return -1;
  }

  cv::Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  M1 *= scaleFactor;
  M2 *= scaleFactor;

  fs.open(extrinsic_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    printf("Failed to open file extrinsic_filename\n");
    return -1;
  }

  fs["R"] >> R;
  fs["T"] >> T;

  stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1,
                &roi2);

  // cv::Mat map11, map12, map21, map22;
  initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

  return 1;
}

int stereoRectification(const std::string intrinsic_filename,
                        const std::string extrinsic_filename,
                        const std::string preprocessing_filename,
                        cv::Mat& P1,
                        cv::Mat& P2,
                        cv::Mat& map11,
                        cv::Mat& map12,
                        cv::Mat& map21,
                        cv::Mat& map22,
                        cv::Size& S1,
                        cv::Size& S2,
                        const double scaleFactor = 1.0) {
  // reading intrinsic parameters
  cv::FileStorage fs(preprocessing_filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    printf("Failed to open file intrinsic_filename\n");
    return -1;
  }
  cv::Mat S1m, S2m, B;
  fs["B"] >> B;
  fs["S1"] >> S1m;
  fs["S2"] >> S2m;
  S1 = cv::Size(S1m.at<int>(0, 0), S1m.at<int>(1, 0));
  S2 = cv::Size(S2m.at<int>(0, 0), S2m.at<int>(1, 0));
  return stereoRectification(intrinsic_filename, extrinsic_filename, S1, P1, P2, map11, map12, map21, map22,
                             scaleFactor);
}

int stereoRectification(const std::string intrinsic_filename,
                        const std::string extrinsic_filename,
                        const std::string preprocessing_filename,
                        cv::Mat& P1,
                        cv::Mat& P2,
                        cv::Mat& map11,
                        cv::Mat& map12,
                        cv::Mat& map21,
                        cv::Mat& map22,
                        const double scaleFactor = 1.0) {
  cv::Size S1(0, 0);
  cv::Size S2(0, 0);
  return stereoRectification(intrinsic_filename, extrinsic_filename, preprocessing_filename, P1, P2, map11, map12,
                             map21, map22, S1, S2, scaleFactor);
}

/**
 * @brief The StereoPreprocessor class stores rectification information and is used to rectify images.
 */

class StereoPreprocessor {
 public:
  cv::Mat P1, P2, map11, map12, map21, map22;
  cv::Size sizeFull, sizeCropped;
  int croppingHeight, croppingWidth;
  bool valid = false;

  StereoPreprocessor(const std::string intrinsic_filename,
                     const std::string extrinsic_filename,
                     const std::string preprocessing_filename) {
    valid = false;

    // reading preprocessing_filename parameters
    cv::FileStorage fs(preprocessing_filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      printf("Failed to open file preprocessing_filename\n");
      return;  // Throw Exception ?
    }
    cv::Mat B;
    fs["B"] >> B;
    croppingHeight = B.at<int>(0, 0);
    croppingWidth = B.at<int>(1, 0);
    cv::Size S1, S2;
    if (!lsfm::stereoRectification(intrinsic_filename, extrinsic_filename, preprocessing_filename, P1, P2, map11, map12,
                                   map21, map22, S1, S2)) {
      std::cout << "Can not open files " << std::endl;
      return;  // Throw Exception ?
    }
    std::cout << "map11" << map11.size() << std::endl;
    std::cout << "map12" << map12.size() << std::endl;

    sizeFull = S1;
    sizeCropped = cv::Size(S1.width - croppingWidth * 2, S1.height - croppingHeight * 2);
    valid = true;
  }

  bool isValid() { return valid; }

  cv::Mat preprocessImage(const cv::Mat img, const int id) {
    //            cvtColor(img,img,cv::COLOR_RGB2GRAY);
    cv::Mat img1, im_gray;
    if (id == 0) {
      remap(img, img1, map11, map12, cv::INTER_LINEAR);  // undistort
    } else if (id == 1) {
      remap(img, img1, map21, map22, cv::INTER_LINEAR);  // undistort
    } else {
      // Throw exception
    }
    im_gray = img1(cv::Rect(croppingWidth, croppingHeight, img1.size().width - croppingWidth * 2,
                            img1.size().height - croppingHeight * 2))
                  .clone();  // crop black borders
    return im_gray;
  }
};

}  // namespace lsfm

#endif
#endif
