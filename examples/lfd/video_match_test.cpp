/// @file video_match_test.cpp
/// @brief Video line matching with motion descriptors.
///
/// Comprehensive video line matching demonstration:
/// - Stereo video processing with rectification
/// - Motion-based line filtering (MotionLineFilter)
/// - Motion descriptor for temporal consistency
/// - StereoLineMatcher for cross-view matching
/// - 3D line triangulation from stereo matches
///
/// @usage ./video_match_test [intrinsic_file] [extrinsic_file] [video_left] [video_right]
/// @param intrinsic_file Camera intrinsic calibration file
/// @param extrinsic_file Camera extrinsic calibration file
/// @param video_left Left camera video file
/// @param video_right Right camera video file

#include <geometry/draw.hpp>
#include <geometry/line3.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>
#include <lfd/PairwiseLineMatcher.hpp>
#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <lsd/lsd_cc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;


//! Calculates the width of the black border on each side of the image, order: top, bottom, left, right
//! Function has to be changed if any of the Borders reach more than one quater of the width/height into the image
template <class MTYPE>
void findBlackCroppingArea(const cv::Mat img, std::vector<int>& border) {
  cv::Mat upper = img(Rect(img.size().width / 4, 0, img.size().width / 2, img.size().height / 2));
  cv::Mat lower = img(Rect(img.size().width / 4, img.size().height / 2, img.size().width / 2, img.size().height / 2));
  cv::Mat left = img(Rect(0, img.size().height / 4, img.size().width / 2, img.size().height / 2));
  cv::Mat right = img(Rect(img.size().width / 2, img.size().height / 4, img.size().width / 2, img.size().height / 2));

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
                        const Size img_size,
                        const double scaleFactor,
                        cv::Mat& P1,
                        cv::Mat& P2,
                        cv::Mat& map11,
                        cv::Mat& map12,
                        cv::Mat& map21,
                        cv::Mat& map22) {
  cv::Mat Q;
  cv::Mat R, T, R1, R2;
  Rect roi1, roi2;

  // reading intrinsic parameters
  FileStorage fs(intrinsic_filename, FileStorage::READ);
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

  fs.open(extrinsic_filename, FileStorage::READ);
  if (!fs.isOpened()) {
    printf("Failed to open file extrinsic_filename\n");
    return -1;
  }

  fs["R"] >> R;
  fs["T"] >> T;

  stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

  std::cout << "P1: " << P1 << std::endl << "P2: " << P2 << std::endl;

  // cv::Mat map11, map12, map21, map22;
  initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
  return 1;
}

void coutPairVector(std::vector<std::pair<int, int>> pixel) {
  for (size_t i = 0; i < pixel.size(); ++i) {
    std::cout << pixel.at(i).first << " & " << pixel.at(i).second << ", ";
  }
  std::cout << std::endl;
}


int main(int argc, char** argv) {
  std::string filename1 = "../../../Datasets/euroc/t1_3_stereo.avi";
  std::string filename2 = "../../../Datasets/euroc/intrinsics.yml";
  std::string filename3 = "../../../Datasets/euroc/extrinsics.yml";

  if (argc > 2) {
    filename1 = argv[1];
    filename2 = argv[2];
    filename3 = argv[3];
  }

  cv::Mat frames1, im_grayLeft, im_grayRight;
  VideoCapture capture;
  capture = VideoCapture(filename1);
  capture.grab();
  capture.retrieve(frames1, IMREAD_GRAYSCALE);
  cvtColor(frames1, frames1, cv::COLOR_RGB2GRAY);
  std::cout << "type: " << frames1.type() << std::endl;

  double scaleFactor = 1.0;
  Size img_size;
  img_size.width = (frames1.size().width / 2) * scaleFactor;
  img_size.height = frames1.size().height * scaleFactor;

  cv::Mat P1, P2, map11, map12, map21, map22;

  if (!stereoRectification(filename2, filename3, img_size, scaleFactor, P1, P2, map11, map12, map21, map22)) {
    cout << "Can not open files " << endl;
    return -1;
  }

  // find black borders
  cv::Mat borderDetect = cv::Mat::ones(frames1.rows, frames1.cols / 2, frames1.type());
  borderDetect *= 255;
  cv::Mat bordersRecti;
  remap(borderDetect, bordersRecti, map11, map12, INTER_LINEAR);  // Undistort
  std::vector<int> border;
  findBlackCroppingArea<uchar>(bordersRecti, border);

  borderDetect = cv::Mat::ones(frames1.rows, frames1.cols / 2, frames1.type());
  borderDetect *= 255;
  remap(borderDetect, bordersRecti, map21, map22, INTER_LINEAR);  // Undistort
  findBlackCroppingArea<uchar>(bordersRecti, border);

  int croppingHeight = std::max(border[0], std::max(border[1], std::max(border[4], border[5])));
  int croppingWidth = std::max(border[2], std::max(border[3], std::max(border[6], border[7])));


  // GaussianBlur(src1, src1, Size(3, 3), 0.6);

  typedef float MyType;

  LsdCC<MyType> lsd1(0.004, 0.008, 20, 0, 3);
  LsdCC<MyType> lsd2(0.004, 0.008, 20, 0, 3);

  im_grayLeft =
      frames1(cv::Range(0, frames1.size().height),
              cv::Range(0, frames1.size().width /
                               2));  // no data copying here - Range excludes the last value of width, thus correct.
  im_grayRight = frames1(cv::Range(0, frames1.size().height),
                         cv::Range(frames1.size().width / 2, frames1.size().width));  // no data copying here

  cv::Mat img1r, img2r;
  remap(im_grayLeft, img1r, map11, map12, INTER_LINEAR);  // Undistort
  remap(im_grayRight, img2r, map21, map22, INTER_LINEAR);

  im_grayLeft = img1r(Rect(croppingWidth, croppingHeight, img1r.size().width - croppingWidth * 2,
                           img1r.size().height - croppingHeight * 2))
                    .clone();
  im_grayRight = img2r(Rect(croppingWidth, croppingHeight, img2r.size().width - croppingWidth * 2,
                            img2r.size().height - croppingHeight * 2))
                     .clone();

  std::vector<FeatureMatch<MyType>> matches;
  std::vector<DescriptorMatch<MyType>> motionMatches, motionMatchesRes;
  std::cout << "width: " << im_grayLeft.size().width << " height: " << im_grayLeft.size().height << std::endl;
  MotionLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> mlf(200, 20, im_grayLeft.size().width,
                                                                 im_grayLeft.size().height);
  std::vector<size_t> maskN, maskP;

  LsdCC<MyType>::LineSegmentVector previousLeftLines, previousRightLines;
  cv::Mat previousImGrayLeft = im_grayLeft;

  std::cout << (8 % 8) << "  " << (9 % 8) << "  " << (7 % 8) << "  " << (-2 % 8) << "  " << (-0 % 8) << std::endl;


  int last_key_press = 0;
  while (last_key_press != 'q') {
    im_grayLeft =
        frames1(cv::Range(0, frames1.size().height),
                cv::Range(0, frames1.size().width /
                                 2));  // no data copying here - Range excludes the last value of width, so correct.
    im_grayRight = frames1(cv::Range(0, frames1.size().height),
                           cv::Range(frames1.size().width / 2, frames1.size().width));  // no data copying here
    /*
            resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height *
       scaleFactor)); resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor,
       im_grayRight.size().height * scaleFactor));
    */
    remap(im_grayLeft, img1r, map11, map12, INTER_LINEAR);  // Undistort
    remap(im_grayRight, img2r, map21, map22, INTER_LINEAR);

    im_grayLeft = img1r(Rect(croppingWidth, croppingHeight, img1r.size().width - croppingWidth * 2,
                             img1r.size().height - croppingHeight * 2))
                      .clone();
    im_grayRight = img2r(Rect(croppingWidth, croppingHeight, img2r.size().width - croppingWidth * 2,
                              img2r.size().height - croppingHeight * 2))
                       .clone();

    lsd1.detect(im_grayLeft);
    lsd2.detect(im_grayRight);

    MatMap dataL, dataR;
    dataL["gx"] = lsd1.imageData()[0];
    dataL["gy"] = lsd1.imageData()[1];
    dataL["img"] = im_grayLeft;

    dataR["gx"] = lsd2.imageData()[0];
    dataR["gy"] = lsd2.imageData()[1];
    dataR["img"] = im_grayRight;

    // typedef GchGradImgInterpolate<MyType,1,2,NoAlign<MyType>> MyGchHelper;
    typedef GchGradImgInterpolate<MyType> MyGchHelper;

    typedef FdcGenericLR<MyType, LsdCC<MyType>::LineSegment, MyGchHelper> MyFdc;


    // Stereo matching ---------------------------------------------------------
    MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
    MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);
    StereoLineMatcher<MyType, LsdCC<MyType>::LineSegmentVector, MyGchHelper> slm(fdcL, fdcR, im_grayLeft.rows,
                                                                                 im_grayLeft.cols / 2, 5, 0.6);

    std::vector<DescriptorMatch<MyType>> bfmatches2, bfmatches3;
    slm.match(lsd1.lineSegments(), lsd2.lineSegments(), bfmatches3);

    MyType mean = 0;
    for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyType>& data) { mean += data.distance; });

    mean /= bfmatches3.size();
    std::cout << "mean distance: " << mean << std::endl;
    for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyType>& data) {
      // mean += data.distance;
      // if (data.distance < mean)
      // if (data.distance < 500)
      bfmatches2.push_back(data);
    });

    std::cout << "final matches: " << bfmatches2.size() << std::endl;
    imshow("Detected Stereo matches",
           drawMatches<MyType, DescriptorMatch<MyType>>(im_grayLeft, lsd1.lineSegments(), im_grayRight,
                                                        lsd2.lineSegments(), bfmatches2));


    // Tracking ---------------------------------------------------------

    typedef FdcLBD<MyType, LsdCC<MyType>::LineSegment, short, FastRoundNearestInterpolator<MyType, short>> LbdFdc;
    [[maybe_unused]] LbdFdc::FdcPtr fdc1 = LbdFdc::createFdc(lsd1.imageData()[0], lsd1.imageData()[1], 7, 5);
    [[maybe_unused]] LbdFdc::FdcPtr fdc2 = LbdFdc::createFdc(lsd2.imageData()[0], lsd2.imageData()[1], 7, 5);


    typedef FdcMotion<MyType, LsdCC<MyType>::LineSegment> MyMotionFdc;
    [[maybe_unused]] MyMotionFdc::FdcPtr fdcM1 = MyMotionFdc::createFdc();
    [[maybe_unused]] MyMotionFdc::FdcPtr fdcM2 = MyMotionFdc::createFdc();


    std::cout << "width: " << im_grayLeft.size().width << " height: " << im_grayLeft.size().height << std::endl;
    MotionLineMatcher<MyType, LsdCC<MyType>::LineSegmentVector, MyMotionFdc> mlm(fdcM1, fdcM2, im_grayLeft.cols,
                                                                                 im_grayLeft.rows);


    double start = double(getTickCount());
    std::pair<MyType, MyType> movement(0, 0);

    mlf.create(lsd1.lineSegments(), previousLeftLines, movement, matches, maskN, maskP);

    double end = double(getTickCount());
    std::cout << "stereo line filter time: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;
    std::cout << "lines left: " << lsd1.lineSegments().size() << ", lines right: " << lsd2.lineSegments().size()
              << std::endl;
    // size_t num = matches.size();
    size_t num = matches.size();

    std::cout << "reduced candidates from " << lsd1.lineSegments().size() * lsd2.lineSegments().size() << " to " << num
              << std::endl;


    start = double(getTickCount());
    motionMatches.clear();
    motionMatchesRes.clear();
    mlm.match(lsd1.lineSegments(), previousLeftLines, movement, motionMatches);
    // mlm.match(previousLeftLines, lsd1.lineSegments(), movement, motionMatches);

    /*
            PairwiseLineMatcher<MyType, typename LbdFdc::descriptor_type> pmatcher;
            if(motionMatches.size() > 1)
                pmatcher.match1D(lsd1.lineSegments(), lsd2.lineSegments(), mlm.getDscNew(), mlm.getDscPrev(),
       motionMatches, motionMatchesRes);
    */

    end = double(getTickCount());
    std::cout << "mlm matches: " << motionMatches.size()
              << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    imshow("Detected tracking matches",
           drawMatches<MyType, DescriptorMatch<MyType>>(im_grayLeft, lsd1.lineSegments(), previousImGrayLeft,
                                                        previousLeftLines, motionMatches));
    // imshow("Detected matches", drawMatches<MyType,DescriptorMatch<MyType>>(previousImGrayLeft, previousLeftLines,
    // im_grayLeft, lsd1.lineSegments(), motionMatches));


    cv::Mat linesPrevLeft = previousImGrayLeft;
    cv::Mat linesLeft = im_grayLeft.clone();
    cv::Mat linesRight = im_grayRight.clone();
    drawLines<MyType>(linesLeft, lsd1.lineSegments());
    drawLines<MyType>(linesRight, lsd2.lineSegments());
    drawLines<MyType>(linesPrevLeft, previousLeftLines);
    imshow("leftLines", linesLeft);
    imshow("rightLeft", linesRight);
    imshow("prevLeftLines", linesPrevLeft);


    // get new frame
    if (!capture.grab()) break;
    capture.retrieve(frames1, IMREAD_GRAYSCALE);
    cvtColor(frames1, frames1, cv::COLOR_RGB2GRAY);

    previousImGrayLeft = im_grayLeft;
    previousLeftLines = lsd1.lineSegments();
    previousRightLines = lsd2.lineSegments();

    waitKey(100000);
    last_key_press = waitKey(1) % 256;
  }


  return 0;
}
