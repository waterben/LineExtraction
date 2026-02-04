/*IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

 By downloading, copying, installing or using the software you agree to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library

Copyright (C) 2011-2012, Lilian Zhang, all rights reserved.
Copyright (C) 2013, Manuele Tamburrano, Stefano Fabri, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include "lsd_edlz.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#define Horizontal 255  // if |dx|<|dy|;
#define Vertical 0      // if |dy|<=|dx|;
#define UpDir 1
#define RightDir 2
#define DownDir 3
#define LeftDir 4
#define TryTime 6
#define SkipEdgePoint 2

using namespace std;
EDLineDetector::EDLineDetector() { InitEDLine_(); }
EDLineDetector::EDLineDetector(EDLineParam param) {
  gradienThreshold_ = static_cast<short>(std::lround(param.gradientThreshold));
  anchorThreshold_ = static_cast<unsigned char>(std::lround(param.anchorThreshold));
  scanIntervals_ = static_cast<unsigned int>(std::max(param.scanIntervals, 1));
  minLineLen_ = std::max(param.minLineLen, 1);
  lineFitErrThreshold_ = param.lineFitErrThreshold;
  bValidate_ = param.validate;
  InitEDLine_();
}
void EDLineDetector::InitEDLine_() {
  ATA = cv::Mat_<int>(2, 2);
  ATV = cv::Mat_<int>(1, 2);
  tempMatLineFit = cv::Mat_<int>(2, 2);
  tempVecLineFit = cv::Mat_<int>(1, 2);
  fitMatT = cv::Mat_<int>(2, minLineLen_);
  fitVec = cv::Mat_<int>(1, minLineLen_);
  for (int i = 0; i < minLineLen_; i++) {
    fitMatT[1][i] = 1;
  }
  dxImg_.create(1, 1, CV_16SC1);
  dyImg_.create(1, 1, CV_16SC1);
  gImgWO_.create(1, 1, CV_8SC1);
  pFirstPartEdgeX_ = NULL;
  pFirstPartEdgeY_ = NULL;
  pFirstPartEdgeS_ = NULL;
  pSecondPartEdgeX_ = NULL;
  pSecondPartEdgeY_ = NULL;
  pSecondPartEdgeS_ = NULL;
  pAnchorX_ = NULL;
  pAnchorY_ = NULL;
}
EDLineDetector::~EDLineDetector() {
  if (pFirstPartEdgeX_ != NULL) {
    delete[] pFirstPartEdgeX_;
    delete[] pFirstPartEdgeY_;
    delete[] pSecondPartEdgeX_;
    delete[] pSecondPartEdgeY_;
    delete[] pAnchorX_;
    delete[] pAnchorY_;
  }
  if (pFirstPartEdgeS_ != NULL) {
    delete[] pFirstPartEdgeS_;
    delete[] pSecondPartEdgeS_;
  }
}

EDLineParam EDLineDetector::getParams() const {
  EDLineParam ret;
  ret.anchorThreshold = static_cast<float>(anchorThreshold_);
  ret.gradientThreshold = static_cast<float>(gradienThreshold_);
  ret.lineFitErrThreshold = lineFitErrThreshold_;
  ret.minLineLen = minLineLen_;
  ret.scanIntervals = static_cast<int>(scanIntervals_);
  ret.validate = bValidate_;
  return ret;
}

void EDLineDetector::setParams(const EDLineParam& param) {
  gradienThreshold_ = static_cast<short>(std::lround(param.gradientThreshold));
  anchorThreshold_ = static_cast<unsigned char>(std::lround(param.anchorThreshold));
  scanIntervals_ = static_cast<unsigned int>(std::max(param.scanIntervals, 1));
  minLineLen_ = std::max(param.minLineLen, 1);
  lineFitErrThreshold_ = param.lineFitErrThreshold;
  bValidate_ = param.validate;
  fitMatT = cv::Mat_<int>(2, minLineLen_);
  fitVec = cv::Mat_<int>(1, minLineLen_);
  for (int i = 0; i < minLineLen_; i++) {
    fitMatT[1][i] = 1;
  }
}

void writeMat(cv::Mat m, string name, int n) {
  std::stringstream ss;
  string s;
  ss << n;
  ss >> s;
  string fileNameConf = name + s;
  cv::FileStorage fsConf(fileNameConf, cv::FileStorage::WRITE);
  fsConf << "m" << m;

  fsConf.release();
}

int EDLineDetector::EdgeDrawing(cv::Mat& image, EdgeChains& edgeChains) {
  imageWidth = static_cast<unsigned int>(image.cols);
  imageHeight = static_cast<unsigned int>(image.rows);
  unsigned int pixelNum = imageWidth * imageHeight;

  unsigned int edgePixelArraySize = pixelNum / 2;
  unsigned int maxNumOfEdge = edgePixelArraySize / 5;
  const unsigned int minLineLenUnsigned = static_cast<unsigned int>(minLineLen_);
  // compute dx, dy images
  if (gImg_.cols != static_cast<int>(imageWidth) || gImg_.rows != static_cast<int>(imageHeight)) {
    if (pFirstPartEdgeX_ != NULL) {
      delete[] pFirstPartEdgeX_;
      delete[] pFirstPartEdgeY_;
      delete[] pSecondPartEdgeX_;
      delete[] pSecondPartEdgeY_;
      delete[] pFirstPartEdgeS_;
      delete[] pSecondPartEdgeS_;
      delete[] pAnchorX_;
      delete[] pAnchorY_;
    }
    dxImg_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_16SC1);
    dyImg_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_16SC1);
    gImgWO_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_16SC1);
    gImg_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_16SC1);
    dirImg_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_8UC1);
    edgeImage_.create(static_cast<int>(imageHeight), static_cast<int>(imageWidth), CV_8UC1);
    pFirstPartEdgeX_ = new unsigned int[edgePixelArraySize];
    pFirstPartEdgeY_ = new unsigned int[edgePixelArraySize];
    pSecondPartEdgeX_ = new unsigned int[edgePixelArraySize];
    pSecondPartEdgeY_ = new unsigned int[edgePixelArraySize];
    pFirstPartEdgeS_ = new unsigned int[maxNumOfEdge];
    pSecondPartEdgeS_ = new unsigned int[maxNumOfEdge];
    pAnchorX_ = new unsigned int[edgePixelArraySize];
    pAnchorY_ = new unsigned int[edgePixelArraySize];
  }
  cv::Sobel(image, dxImg_, CV_16SC1, 1, 0, 3);
  cv::Sobel(image, dyImg_, CV_16SC1, 0, 1, 3);


  // compute gradient and direction images

  cv::Mat dxABS_m = cv::abs(dxImg_);
  cv::Mat dyABS_m = cv::abs(dyImg_);
  cv::Mat sumDxDy;
  cv::add(dyABS_m, dxABS_m, sumDxDy);


  cv::threshold(sumDxDy, gImg_, gradienThreshold_ + 1, 255, cv::THRESH_TOZERO);
  gImg_ = gImg_ / 4;
  gImgWO_ = sumDxDy / 4;
  cv::compare(dxABS_m, dyABS_m, dirImg_, cv::CMP_LT);

  short* pgImg = gImg_.ptr<short>();
  unsigned char* pdirImg = dirImg_.ptr();

  // extract the anchors in the gradient image, store into a vector
  memset(pAnchorX_, 0, edgePixelArraySize * sizeof(unsigned int));  // initialization
  memset(pAnchorY_, 0, edgePixelArraySize * sizeof(unsigned int));
  unsigned int anchorsSize = 0;
  unsigned int indexInArray = 0;
  short gValue1 = 0;
  short gValue2 = 0;
  short gValue3 = 0;
  for (unsigned int w = 1; w < imageWidth - 1; w = w + scanIntervals_) {
    for (unsigned int h = 1; h < imageHeight - 1; h = h + scanIntervals_) {
      indexInArray = h * imageWidth + w;
      // gValue1 = pdirImg[indexInArray];
      if (pdirImg[indexInArray] ==
          Horizontal) {  // if the direction of pixel is horizontal, then compare with up and down
        // gValue2 = pgImg[indexInArray];
        if (pgImg[indexInArray] >= pgImg[indexInArray - imageWidth] + anchorThreshold_ &&
            pgImg[indexInArray] >=
                pgImg[indexInArray + imageWidth] + anchorThreshold_) {  // (w,h) is accepted as an anchor
          pAnchorX_[anchorsSize] = w;
          pAnchorY_[anchorsSize++] = h;
        }
      } else {  // if(pdirImg[indexInArray]==Vertical){//it is vertical edge, should be compared with left and right
        // gValue2 = pgImg[indexInArray];
        if (pgImg[indexInArray] >= pgImg[indexInArray - 1] + anchorThreshold_ &&
            pgImg[indexInArray] >= pgImg[indexInArray + 1] + anchorThreshold_) {  // (w,h) is accepted as an anchor
          pAnchorX_[anchorsSize] = w;
          pAnchorY_[anchorsSize++] = h;
        }
      }
    }
  }
  if (anchorsSize > edgePixelArraySize) {
    cout << "anchor size is larger than its maximal size. anchorsSize=" << anchorsSize
         << ", maximal size = " << edgePixelArraySize << endl;
    return -1;
  }

  // link the anchors by smart routing
  edgeImage_.setTo(0);
  unsigned char* pEdgeImg = edgeImage_.data;
  memset(pFirstPartEdgeX_, 0, edgePixelArraySize * sizeof(unsigned int));  // initialization
  memset(pFirstPartEdgeY_, 0, edgePixelArraySize * sizeof(unsigned int));
  memset(pSecondPartEdgeX_, 0, edgePixelArraySize * sizeof(unsigned int));
  memset(pSecondPartEdgeY_, 0, edgePixelArraySize * sizeof(unsigned int));
  memset(pFirstPartEdgeS_, 0, maxNumOfEdge * sizeof(unsigned int));
  memset(pSecondPartEdgeS_, 0, maxNumOfEdge * sizeof(unsigned int));
  unsigned int offsetPFirst = 0, offsetPSecond = 0;
  unsigned int offsetPS = 0;

  unsigned int x, y;
  unsigned int lastX = 0;
  unsigned int lastY = 0;
  unsigned char lastDirection;      // up = 1, right = 2, down = 3, left = 4;
  unsigned char shouldGoDirection;  // up = 1, right = 2, down = 3, left = 4;
  unsigned int edgeLenFirst = 0;
  unsigned int edgeLenSecond = 0;
  for (unsigned int i = 0; i < anchorsSize; i++) {
    x = pAnchorX_[i];
    y = pAnchorY_[i];
    lastX = x;
    lastY = y;
    indexInArray = y * imageWidth + x;
    if (pEdgeImg[indexInArray]) {  // if anchor i is already been an edge pixel.
      continue;
    }
    /*The walk stops under 3 conditions:
     * 1. We move out of the edge areas, i.e., the thresholded gradient value
     *    of the current pixel is 0.
     * 2. The current direction of the edge changes, i.e., from horizontal
     *    to vertical or vice versa.?? (This is turned out not correct. From the online edge draw demo
     *    we can figure out that authors don't implement this rule either because their extracted edge
     *    chain could be a circle which means pixel directions would definitely be different
     *    in somewhere on the chain.)
     * 3. We encounter a previously detected edge pixel. */
    pFirstPartEdgeS_[offsetPS] = offsetPFirst;
    if (pdirImg[indexInArray] == Horizontal) {  // if the direction of this pixel is horizontal, then go left and right.
      // fist go right, pixel direction may be different during linking.
      lastDirection = RightDir;
      while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray]) {
        pEdgeImg[indexInArray] = 1;  // Mark this pixel as an edge pixel
        pFirstPartEdgeX_[offsetPFirst] = x;
        pFirstPartEdgeY_[offsetPFirst++] = y;
        shouldGoDirection = 0;                                       // unknown
        if (pdirImg[indexInArray] == Horizontal) {                   // should go left or right
          if (lastDirection == UpDir || lastDirection == DownDir) {  // change the pixel direction now
            if (x > lastX) {                                         // should go right
              shouldGoDirection = RightDir;
            } else {  // should go left
              shouldGoDirection = LeftDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == RightDir || shouldGoDirection == RightDir) {  // go right
            if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1) {     // reach the image border
              break;
            }
            // Look at 3 neighbors to the right and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray + 1];
            gValue3 = pgImg[indexInArray + imageWidth + 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-right
              x = x + 1;
              y = y + 1;
            } else {  // straight-right
              x = x + 1;
            }
            lastDirection = RightDir;
          } else if (lastDirection == LeftDir || shouldGoDirection == LeftDir) {  // go left
            if (x == 0 || y == 0 || y == imageHeight - 1) {                       // reach the image border
              break;
            }
            // Look at 3 neighbors to the left and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth - 1];
            gValue2 = pgImg[indexInArray - 1];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-left
              x = x - 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-left
              x = x - 1;
            }
            lastDirection = LeftDir;
          }
        } else {                                                        // should go up or down.
          if (lastDirection == RightDir || lastDirection == LeftDir) {  // change the pixel direction now
            if (y > lastY) {                                            // should go down
              shouldGoDirection = DownDir;
            } else {  // should go up
              shouldGoDirection = UpDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == DownDir || shouldGoDirection == DownDir) {  // go down
            if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1) {   // reach the image border
              break;
            }
            // Look at 3 neighbors to the down and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray + imageWidth + 1];
            gValue2 = pgImg[indexInArray + imageWidth];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // down-right
              x = x + 1;
              y = y + 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-down
              y = y + 1;
            }
            lastDirection = DownDir;
          } else if (lastDirection == UpDir || shouldGoDirection == UpDir) {  // go up
            if (x == 0 || x == imageWidth - 1 || y == 0) {                    // reach the image border
              break;
            }
            // Look at 3 neighbors to the up and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray - imageWidth];
            gValue3 = pgImg[indexInArray - imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // up-left
              x = x - 1;
              y = y - 1;
            } else {  // straight-up
              y = y - 1;
            }
            lastDirection = UpDir;
          }
        }
        indexInArray = y * imageWidth + x;
      }  // end while go right
      // then go left, pixel direction may be different during linking.
      x = pAnchorX_[i];
      y = pAnchorY_[i];
      indexInArray = y * imageWidth + x;
      pEdgeImg[indexInArray] = 0;  // mark the anchor point be a non-edge pixel and
      lastDirection = LeftDir;
      pSecondPartEdgeS_[offsetPS] = offsetPSecond;
      while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray]) {
        pEdgeImg[indexInArray] = 1;  // Mark this pixel as an edge pixel
        pSecondPartEdgeX_[offsetPSecond] = x;
        pSecondPartEdgeY_[offsetPSecond++] = y;
        shouldGoDirection = 0;                                       // unknown
        if (pdirImg[indexInArray] == Horizontal) {                   // should go left or right
          if (lastDirection == UpDir || lastDirection == DownDir) {  // change the pixel direction now
            if (x > lastX) {                                         // should go right
              shouldGoDirection = RightDir;
            } else {  // should go left
              shouldGoDirection = LeftDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == RightDir || shouldGoDirection == RightDir) {  // go right
            if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1) {     // reach the image border
              break;
            }
            // Look at 3 neighbors to the right and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray + 1];
            gValue3 = pgImg[indexInArray + imageWidth + 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-right
              x = x + 1;
              y = y + 1;
            } else {  // straight-right
              x = x + 1;
            }
            lastDirection = RightDir;
          } else if (lastDirection == LeftDir || shouldGoDirection == LeftDir) {  // go left
            if (x == 0 || y == 0 || y == imageHeight - 1) {                       // reach the image border
              break;
            }
            // Look at 3 neighbors to the left and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth - 1];
            gValue2 = pgImg[indexInArray - 1];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-left
              x = x - 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-left
              x = x - 1;
            }
            lastDirection = LeftDir;
          }
        } else {                                                        // should go up or down.
          if (lastDirection == RightDir || lastDirection == LeftDir) {  // change the pixel direction now
            if (y > lastY) {                                            // should go down
              shouldGoDirection = DownDir;
            } else {  // should go up
              shouldGoDirection = UpDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == DownDir || shouldGoDirection == DownDir) {  // go down
            if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1) {   // reach the image border
              break;
            }
            // Look at 3 neighbors to the down and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray + imageWidth + 1];
            gValue2 = pgImg[indexInArray + imageWidth];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // down-right
              x = x + 1;
              y = y + 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-down
              y = y + 1;
            }
            lastDirection = DownDir;
          } else if (lastDirection == UpDir || shouldGoDirection == UpDir) {  // go up
            if (x == 0 || x == imageWidth - 1 || y == 0) {                    // reach the image border
              break;
            }
            // Look at 3 neighbors to the up and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray - imageWidth];
            gValue3 = pgImg[indexInArray - imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // up-left
              x = x - 1;
              y = y - 1;
            } else {  // straight-up
              y = y - 1;
            }
            lastDirection = UpDir;
          }
        }
        indexInArray = y * imageWidth + x;
      }  // end while go left
      // end anchor is Horizontal
    } else {  // the direction of this pixel is vertical, go up and down
      // fist go down, pixel direction may be different during linking.
      lastDirection = DownDir;
      while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray]) {
        pEdgeImg[indexInArray] = 1;  // Mark this pixel as an edge pixel
        pFirstPartEdgeX_[offsetPFirst] = x;
        pFirstPartEdgeY_[offsetPFirst++] = y;
        shouldGoDirection = 0;                                       // unknown
        if (pdirImg[indexInArray] == Horizontal) {                   // should go left or right
          if (lastDirection == UpDir || lastDirection == DownDir) {  // change the pixel direction now
            if (x > lastX) {                                         // should go right
              shouldGoDirection = RightDir;
            } else {  // should go left
              shouldGoDirection = LeftDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == RightDir || shouldGoDirection == RightDir) {  // go right
            if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1) {     // reach the image border
              break;
            }
            // Look at 3 neighbors to the right and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray + 1];
            gValue3 = pgImg[indexInArray + imageWidth + 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-right
              x = x + 1;
              y = y + 1;
            } else {  // straight-right
              x = x + 1;
            }
            lastDirection = RightDir;
          } else if (lastDirection == LeftDir || shouldGoDirection == LeftDir) {  // go left
            if (x == 0 || y == 0 || y == imageHeight - 1) {                       // reach the image border
              break;
            }
            // Look at 3 neighbors to the left and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth - 1];
            gValue2 = pgImg[indexInArray - 1];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-left
              x = x - 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-left
              x = x - 1;
            }
            lastDirection = LeftDir;
          }
        } else {                                                        // should go up or down.
          if (lastDirection == RightDir || lastDirection == LeftDir) {  // change the pixel direction now
            if (y > lastY) {                                            // should go down
              shouldGoDirection = DownDir;
            } else {  // should go up
              shouldGoDirection = UpDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == DownDir || shouldGoDirection == DownDir) {  // go down
            if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1) {   // reach the image border
              break;
            }
            // Look at 3 neighbors to the down and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray + imageWidth + 1];
            gValue2 = pgImg[indexInArray + imageWidth];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // down-right
              x = x + 1;
              y = y + 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-down
              y = y + 1;
            }
            lastDirection = DownDir;
          } else if (lastDirection == UpDir || shouldGoDirection == UpDir) {  // go up
            if (x == 0 || x == imageWidth - 1 || y == 0) {                    // reach the image border
              break;
            }
            // Look at 3 neighbors to the up and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray - imageWidth];
            gValue3 = pgImg[indexInArray - imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // up-left
              x = x - 1;
              y = y - 1;
            } else {  // straight-up
              y = y - 1;
            }
            lastDirection = UpDir;
          }
        }
        indexInArray = y * imageWidth + x;
      }  // end while go down
      // then go up, pixel direction may be different during linking.
      lastDirection = UpDir;
      x = pAnchorX_[i];
      y = pAnchorY_[i];
      indexInArray = y * imageWidth + x;
      pEdgeImg[indexInArray] = 0;  // mark the anchor point be a non-edge pixel and
      pSecondPartEdgeS_[offsetPS] = offsetPSecond;
      while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray]) {
        pEdgeImg[indexInArray] = 1;  // Mark this pixel as an edge pixel
        pSecondPartEdgeX_[offsetPSecond] = x;
        pSecondPartEdgeY_[offsetPSecond++] = y;
        shouldGoDirection = 0;                                       // unknown
        if (pdirImg[indexInArray] == Horizontal) {                   // should go left or right
          if (lastDirection == UpDir || lastDirection == DownDir) {  // change the pixel direction now
            if (x > lastX) {                                         // should go right
              shouldGoDirection = RightDir;
            } else {  // should go left
              shouldGoDirection = LeftDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == RightDir || shouldGoDirection == RightDir) {  // go right
            if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1) {     // reach the image border
              break;
            }
            // Look at 3 neighbors to the right and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray + 1];
            gValue3 = pgImg[indexInArray + imageWidth + 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-right
              x = x + 1;
              y = y + 1;
            } else {  // straight-right
              x = x + 1;
            }
            lastDirection = RightDir;
          } else if (lastDirection == LeftDir || shouldGoDirection == LeftDir) {  // go left
            if (x == 0 || y == 0 || y == imageHeight - 1) {                       // reach the image border
              break;
            }
            // Look at 3 neighbors to the left and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth - 1];
            gValue2 = pgImg[indexInArray - 1];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-left
              x = x - 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-left
              x = x - 1;
            }
            lastDirection = LeftDir;
          }
        } else {                                                        // should go up or down.
          if (lastDirection == RightDir || lastDirection == LeftDir) {  // change the pixel direction now
            if (y > lastY) {                                            // should go down
              shouldGoDirection = DownDir;
            } else {  // should go up
              shouldGoDirection = UpDir;
            }
          }
          lastX = x;
          lastY = y;
          if (lastDirection == DownDir || shouldGoDirection == DownDir) {  // go down
            if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1) {   // reach the image border
              break;
            }
            // Look at 3 neighbors to the down and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray + imageWidth + 1];
            gValue2 = pgImg[indexInArray + imageWidth];
            gValue3 = pgImg[indexInArray + imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // down-right
              x = x + 1;
              y = y + 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // down-left
              x = x - 1;
              y = y + 1;
            } else {  // straight-down
              y = y + 1;
            }
            lastDirection = DownDir;
          } else if (lastDirection == UpDir || shouldGoDirection == UpDir) {  // go up
            if (x == 0 || x == imageWidth - 1 || y == 0) {                    // reach the image border
              break;
            }
            // Look at 3 neighbors to the up and pick the one with the max. gradient value
            gValue1 = pgImg[indexInArray - imageWidth + 1];
            gValue2 = pgImg[indexInArray - imageWidth];
            gValue3 = pgImg[indexInArray - imageWidth - 1];
            if (gValue1 >= gValue2 && gValue1 >= gValue3) {  // up-right
              x = x + 1;
              y = y - 1;
            } else if (gValue3 >= gValue2 && gValue3 >= gValue1) {  // up-left
              x = x - 1;
              y = y - 1;
            } else {  // straight-up
              y = y - 1;
            }
            lastDirection = UpDir;
          }
        }
        indexInArray = y * imageWidth + x;
      }  // end while go up
    }  // end anchor is Vertical
    // only keep the edge chains whose length is larger than the minLineLen_;
    edgeLenFirst = offsetPFirst - pFirstPartEdgeS_[offsetPS];
    edgeLenSecond = offsetPSecond - pSecondPartEdgeS_[offsetPS];
    if (edgeLenFirst + edgeLenSecond < minLineLenUnsigned + 1U) {  // short edge, drop it
      offsetPFirst = pFirstPartEdgeS_[offsetPS];
      offsetPSecond = pSecondPartEdgeS_[offsetPS];
    } else {
      offsetPS++;
    }
  }
  // store the last index
  pFirstPartEdgeS_[offsetPS] = offsetPFirst;
  pSecondPartEdgeS_[offsetPS] = offsetPSecond;
  if (offsetPS > maxNumOfEdge) {
    cout << "Edge drawing Error: The total number of edges is larger than MaxNumOfEdge, "
            "numofedge = "
         << offsetPS << ", MaxNumOfEdge=" << maxNumOfEdge << endl;
    return -1;
  }
  if (offsetPFirst > edgePixelArraySize || offsetPSecond > edgePixelArraySize) {
    cout << "Edge drawing Error: The total number of edge pixels is larger than MaxNumOfEdgePixels, "
            "numofedgePixel1 = "
         << offsetPFirst << ",  numofedgePixel2 = " << offsetPSecond << ", MaxNumOfEdgePixel=" << edgePixelArraySize
         << endl;
    return -1;
  }

  /*now all the edge information are stored in pFirstPartEdgeX_, pFirstPartEdgeY_,
   *pFirstPartEdgeS_,  pSecondPartEdgeX_, pSecondPartEdgeY_, pSecondPartEdgeS_;
   *we should reorganize them into edgeChains for easily using.	*/
  int tempID;
  edgeChains.xCors.resize(offsetPFirst + offsetPSecond);
  edgeChains.yCors.resize(offsetPFirst + offsetPSecond);
  edgeChains.sId.resize(offsetPS + 1);
  unsigned int* pxCors = edgeChains.xCors.data();
  unsigned int* pyCors = edgeChains.yCors.data();
  unsigned int* psId = edgeChains.sId.data();
  offsetPFirst = 0;
  offsetPSecond = 0;
  unsigned int indexInCors = 0;
  unsigned int numOfEdges = 0;
  for (unsigned int edgeId = 0; edgeId < offsetPS; edgeId++) {
    // step1, put the first and second parts edge coordinates together from edge start to edge end
    psId[numOfEdges++] = indexInCors;
    indexInArray = pFirstPartEdgeS_[edgeId];
    offsetPFirst = pFirstPartEdgeS_[edgeId + 1];
    if (offsetPFirst > indexInArray) {
      for (tempID = static_cast<int>(offsetPFirst) - 1; tempID >= static_cast<int>(indexInArray);
           --tempID) {  // add first part edge
        pxCors[indexInCors] = pFirstPartEdgeX_[static_cast<std::size_t>(tempID)];
        pyCors[indexInCors++] = pFirstPartEdgeY_[static_cast<std::size_t>(tempID)];
      }
    }
    indexInArray = pSecondPartEdgeS_[edgeId];
    offsetPSecond = pSecondPartEdgeS_[edgeId + 1];
    for (tempID = static_cast<int>(indexInArray) + 1; tempID < static_cast<int>(offsetPSecond);
         ++tempID) {  // add second part edge
      const auto tempIndex = static_cast<std::size_t>(tempID);
      pxCors[indexInCors] = pSecondPartEdgeX_[tempIndex];
      pyCors[indexInCors++] = pSecondPartEdgeY_[tempIndex];
    }
  }
  psId[numOfEdges] = indexInCors;  // the end index of the last edge
  edgeChains.numOfEdges = numOfEdges;
  return 1;
}


int EDLineDetector::EDline(cv::Mat& image, LineChains& lines) {
  // first, call EdgeDrawing function to extract edges
  EdgeChains edges;
  if (!EdgeDrawing(image, edges)) {
    cout << "Line Detection not finished" << endl;
    return -1;
  }

  // detect lines
  unsigned int linePixelID = edges.sId[edges.numOfEdges];
  lines.xCors.resize(linePixelID);
  lines.yCors.resize(linePixelID);
  lines.sId.resize(5 * edges.numOfEdges);
  unsigned int* pEdgeXCors = edges.xCors.data();
  unsigned int* pEdgeYCors = edges.yCors.data();
  unsigned int* pEdgeSID = edges.sId.data();
  unsigned int* pLineXCors = lines.xCors.data();
  unsigned int* pLineYCors = lines.yCors.data();
  unsigned int* pLineSID = lines.sId.data();
  logNT_ = 2.0 * (std::log10(static_cast<double>(imageWidth)) + std::log10(static_cast<double>(imageHeight)));
  double lineFitErr = 0.0;  // the line fit error;
  std::array<double, 2> lineEquation;
  lineEquations_.clear();
  lineEndpoints_.clear();
  lineDirection_.clear();
  unsigned char* pdirImg = dirImg_.data;
  unsigned int numOfLines = 0;
  unsigned int offsetInEdgeArrayS, offsetInEdgeArrayE, newOffsetS;  // start index and end index
  unsigned int offsetInLineArray = 0;
  float direction;  // line direction
  const unsigned int minLineLenUnsigned = static_cast<unsigned int>(minLineLen_);

  for (unsigned int edgeID = 0; edgeID < edges.numOfEdges; edgeID++) {
    offsetInEdgeArrayS = pEdgeSID[edgeID];
    offsetInEdgeArrayE = pEdgeSID[edgeID + 1];
    while (offsetInEdgeArrayE >
           offsetInEdgeArrayS +
               minLineLenUnsigned) {  // extract line segments from an edge, may find more than one segments
      // find an initial line segment
      while (offsetInEdgeArrayE > offsetInEdgeArrayS + minLineLenUnsigned) {
        lineFitErr = LeastSquaresLineFit_(pEdgeXCors, pEdgeYCors, offsetInEdgeArrayS, lineEquation);
        if (lineFitErr <= lineFitErrThreshold_) break;  // ok, an initial line segment detected
        offsetInEdgeArrayS += SkipEdgePoint;  // skip the first two pixel in the chain and try with the remaining pixels
      }
      if (lineFitErr > lineFitErrThreshold_) break;  // no line is detected
      // An initial line segment is detected. Try to extend this line segment
      pLineSID[numOfLines] = offsetInLineArray;
      double coef1;           // for a line ax+by+c=0, coef1 = 1/sqrt(a^2+b^2);
      double pointToLineDis;  // for a line ax+by+c=0 and a point(xi, yi), pointToLineDis = coef1*|a*xi+b*yi+c|
      bool bExtended = true;
      bool bFirstTry = true;
      unsigned int numOfOutlier = 0;  // to against noise, we accept a few outlier of a line.
      int tryTimes = 0;
      if (pdirImg[pEdgeYCors[offsetInEdgeArrayS] * imageWidth + pEdgeXCors[offsetInEdgeArrayS]] ==
          Horizontal) {  // y=ax+b, i.e. ax-y+b=0
        while (bExtended) {
          tryTimes++;
          if (bFirstTry) {
            bFirstTry = false;
            for (int i = 0; i < minLineLen_; i++) {  // First add the initial line segment to the line array
              pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
              pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
            }
          } else {  // after each try, line is extended, line equation should be re-estimated
            // adjust the line equation
            lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines], newOffsetS,
                                              offsetInLineArray, lineEquation);
          }
          coef1 = 1.0 / std::sqrt(lineEquation[0] * lineEquation[0] + 1.0);
          numOfOutlier = 0;
          newOffsetS = offsetInLineArray;
          while (offsetInEdgeArrayE > offsetInEdgeArrayS) {
            pointToLineDis = std::fabs(lineEquation[0] * pEdgeXCors[offsetInEdgeArrayS] -
                                       pEdgeYCors[offsetInEdgeArrayS] + lineEquation[1]) *
                             coef1;
            pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
            pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
            if (pointToLineDis > lineFitErrThreshold_) {
              numOfOutlier++;
              if (numOfOutlier > 3U) break;
            } else {  // we count number of connective outliers.
              numOfOutlier = 0;
            }
          }
          // pop back the last few outliers from lines and return them to edge chain
          offsetInLineArray -= numOfOutlier;
          offsetInEdgeArrayS -= numOfOutlier;
          if (offsetInLineArray - newOffsetS > 0 && tryTimes < TryTime) {  // some new pixels are added to the line
          } else {
            bExtended = false;  // no new pixels are added.
          }
        }
        // the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
        std::array<double, 3> lineEqu = {lineEquation[0] * coef1, -1 * coef1, lineEquation[1] * coef1};
        if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu,
                            direction)) {  // check the line
          // store the line equation coefficients
          lineEquations_.push_back(lineEqu);
          /*At last, compute the line endpoints and store them.
           *we project the first and last pixels in the pixelChain onto the best fit line
           *to get the line endpoints.
           *xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
           *yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
          std::array<float, 4> lineEndP;  // line endpoints
          double a1 = lineEqu[1] * lineEqu[1];
          double a2 = lineEqu[0] * lineEqu[0];
          double a3 = lineEqu[0] * lineEqu[1];
          double a4 = lineEqu[2] * lineEqu[0];
          double a5 = lineEqu[2] * lineEqu[1];
          unsigned int Px = pLineXCors[pLineSID[numOfLines]];  // first pixel
          unsigned int Py = pLineYCors[pLineSID[numOfLines]];
          lineEndP[0] = static_cast<float>(a1 * Px - a3 * Py - a4);  // x
          lineEndP[1] = static_cast<float>(a2 * Py - a3 * Px - a5);  // y
          Px = pLineXCors[offsetInLineArray - 1];                    // last pixel
          Py = pLineYCors[offsetInLineArray - 1];
          lineEndP[2] = static_cast<float>(a1 * Px - a3 * Py - a4);  // x
          lineEndP[3] = static_cast<float>(a2 * Py - a3 * Px - a5);  // y
          lineEndpoints_.push_back(lineEndP);
          lineDirection_.push_back(direction);
          numOfLines++;
        } else {
          offsetInLineArray = pLineSID[numOfLines];  // line was not accepted, the offset is set back
        }
      } else {  // x=ay+b, i.e. x-ay-b=0
        while (bExtended) {
          tryTimes++;
          if (bFirstTry) {
            bFirstTry = false;
            for (int i = 0; i < minLineLen_; i++) {  // First add the initial line segment to the line array
              pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
              pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
            }
          } else {  // after each try, line is extended, line equation should be re-estimated
            // adjust the line equation
            lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines], newOffsetS,
                                              offsetInLineArray, lineEquation);
          }
          coef1 = 1.0 / std::sqrt(1.0 + lineEquation[0] * lineEquation[0]);
          numOfOutlier = 0;
          newOffsetS = offsetInLineArray;
          while (offsetInEdgeArrayE > offsetInEdgeArrayS) {
            pointToLineDis = std::fabs(pEdgeXCors[offsetInEdgeArrayS] -
                                       lineEquation[0] * pEdgeYCors[offsetInEdgeArrayS] - lineEquation[1]) *
                             coef1;
            pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
            pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
            if (pointToLineDis > lineFitErrThreshold_) {
              numOfOutlier++;
              if (numOfOutlier > 3U) break;
            } else {  // we count number of connective outliers.
              numOfOutlier = 0;
            }
          }
          // pop back the last few outliers from lines and return them to edge chain
          offsetInLineArray -= numOfOutlier;
          offsetInEdgeArrayS -= numOfOutlier;
          if (offsetInLineArray - newOffsetS > 0 && tryTimes < TryTime) {  // some new pixels are added to the line
          } else {
            bExtended = false;  // no new pixels are added.
          }
        }
        // the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
        std::array<double, 3> lineEqu = {1 * coef1, -lineEquation[0] * coef1, -lineEquation[1] * coef1};
        if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu,
                            direction)) {  // check the line
          // store the line equation coefficients
          lineEquations_.push_back(lineEqu);
          /*At last, compute the line endpoints and store them.
           *we project the first and last pixels in the pixelChain onto the best fit line
           *to get the line endpoints.
           *xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
           *yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
          std::array<float, 4> lineEndP;  // line endpoints
          double a1 = lineEqu[1] * lineEqu[1];
          double a2 = lineEqu[0] * lineEqu[0];
          double a3 = lineEqu[0] * lineEqu[1];
          double a4 = lineEqu[2] * lineEqu[0];
          double a5 = lineEqu[2] * lineEqu[1];
          unsigned int Px = pLineXCors[pLineSID[numOfLines]];  // first pixel
          unsigned int Py = pLineYCors[pLineSID[numOfLines]];
          lineEndP[0] = static_cast<float>(a1 * Px - a3 * Py - a4);  // x
          lineEndP[1] = static_cast<float>(a2 * Py - a3 * Px - a5);  // y
          Px = pLineXCors[offsetInLineArray - 1];                    // last pixel
          Py = pLineYCors[offsetInLineArray - 1];
          lineEndP[2] = static_cast<float>(a1 * Px - a3 * Py - a4);  // x
          lineEndP[3] = static_cast<float>(a2 * Py - a3 * Px - a5);  // y
          lineEndpoints_.push_back(lineEndP);
          lineDirection_.push_back(direction);
          numOfLines++;
        } else {
          offsetInLineArray = pLineSID[numOfLines];  // line was not accepted, the offset is set back
        }
      }
      // Extract line segments from the remaining pixel; Current chain has been shortened already.
    }
  }  // end for(unsigned int edgeID=0; edgeID<edges.numOfEdges; edgeID++)
  pLineSID[numOfLines] = offsetInLineArray;
  lines.numOfLines = numOfLines;
  return 1;
}


double EDLineDetector::LeastSquaresLineFit_(unsigned int* xCors,
                                            unsigned int* yCors,
                                            unsigned int offsetS,
                                            std::array<double, 2>& lineEquation) {
  if (lineEquation.size() != 2) {
    std::cout << "SHOULD NOT BE != 2" << std::endl;
    CV_Assert(false);
  }
  float* pMatT;
  float* pATA;
  double fitError = 0.0;
  double coef;
  unsigned char* pdirImg = dirImg_.data;
  /*If the first pixel in this chain is horizontal,
   *then we try to find a horizontal line, y=ax+b;*/
  if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal) {
    /*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
     * [x0,1]         [y0]
     * [x1,1] [a]     [y1]
     *    .   [b]  =   .
     * [xn,1]         [yn]*/
    pMatT = fitMatT.ptr<float>();  // fitMatT = [x0, x1, ... xn; 1,1,...,1];
    for (int i = 0; i < minLineLen_; i++) {
      const unsigned int currentX = xCors[offsetS];
      const unsigned int currentY = yCors[offsetS];
      ++offsetS;
      *(pMatT++) = static_cast<float>(currentX);
      fitVec[0][i] = static_cast<float>(currentY);
    }
    ATA = fitMatT * fitMatT.t();
    ATV = fitMatT * fitVec.t();
    /* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
    pATA = ATA.ptr<float>();
    const double a00 = static_cast<double>(pATA[0]);
    const double a01 = static_cast<double>(pATA[1]);
    const double a10 = static_cast<double>(pATA[2]);
    const double a11 = static_cast<double>(pATA[3]);
    const double v0 = static_cast<double>(ATV[0][0]);
    const double v1 = static_cast<double>(ATV[0][1]);
    const double denom = a00 * a11 - a01 * a10;
    coef = 1.0 / denom;
    //		lineEquation = svd.Invert(ATA) * matT * vec;
    lineEquation[0] = coef * (a11 * v0 - a01 * v1);
    lineEquation[1] = coef * (a00 * v1 - a10 * v0);
    /*compute line fit error */
    unsigned int errorOffset = offsetS - static_cast<unsigned int>(minLineLen_);
    for (int i = 0; i < minLineLen_; i++) {
      const unsigned int xVal = xCors[errorOffset];
      const unsigned int yVal = yCors[errorOffset];
      ++errorOffset;
      coef = static_cast<double>(yVal) - static_cast<double>(xVal) * lineEquation[0] - lineEquation[1];
      fitError += coef * coef;
    }
    return std::sqrt(fitError);
  }
  /*If the first pixel in this chain is vertical,
   *then we try to find a vertical line, x=ay+b;*/
  if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical) {
    /*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
     * [y0,1]         [x0]
     * [y1,1] [a]     [x1]
     *    .   [b]  =   .
     * [yn,1]         [xn]*/
    pMatT = fitMatT.ptr<float>();  // fitMatT = [y0, y1, ... yn; 1,1,...,1];
    for (int i = 0; i < minLineLen_; i++) {
      const unsigned int currentY = yCors[offsetS];
      const unsigned int currentX = xCors[offsetS];
      ++offsetS;
      *(pMatT++) = static_cast<float>(currentY);
      fitVec[0][i] = static_cast<float>(currentX);
    }
    ATA = fitMatT * (fitMatT.t());
    ATV = fitMatT * fitVec.t();
    /* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
    pATA = ATA.ptr<float>();
    const double b00 = static_cast<double>(pATA[0]);
    const double b01 = static_cast<double>(pATA[1]);
    const double b10 = static_cast<double>(pATA[2]);
    const double b11 = static_cast<double>(pATA[3]);
    const double bv0 = static_cast<double>(ATV[0][0]);
    const double bv1 = static_cast<double>(ATV[0][1]);
    const double bDenom = b00 * b11 - b01 * b10;
    coef = 1.0 / bDenom;
    //		lineEquation = svd.Invert(ATA) * matT * vec;
    lineEquation[0] = coef * (b11 * bv0 - b01 * bv1);
    lineEquation[1] = coef * (b00 * bv1 - b10 * bv0);
    /*compute line fit error */
    unsigned int errorOffset = offsetS - static_cast<unsigned int>(minLineLen_);
    for (int i = 0; i < minLineLen_; i++) {
      const unsigned int xVal = xCors[errorOffset];
      const unsigned int yVal = yCors[errorOffset];
      ++errorOffset;
      coef = static_cast<double>(xVal) - static_cast<double>(yVal) * lineEquation[0] - lineEquation[1];
      fitError += coef * coef;
    }
    return std::sqrt(fitError);
  }
  return 0;
}
double EDLineDetector::LeastSquaresLineFit_(unsigned int* xCors,
                                            unsigned int* yCors,
                                            unsigned int offsetS,
                                            unsigned int newOffsetS,
                                            unsigned int offsetE,
                                            std::array<double, 2>& lineEquation) {
  int length = static_cast<int>(offsetE - offsetS);
  int newLength = static_cast<int>(offsetE - newOffsetS);
  if (length <= 0 || newLength <= 0) {
    cout << "EDLineDetector::LeastSquaresLineFit_ Error:"
            " the expected line index is wrong...offsetE = "
         << offsetE << ", offsetS=" << offsetS << ", newOffsetS=" << newOffsetS << endl;
    return -1;
  }
  if (lineEquation.size() != 2) {
    std::cout << "SHOULD NOT BE != 2" << std::endl;
    CV_Assert(false);
  }
  cv::Mat_<float> matT(2, newLength);
  cv::Mat_<float> vec(newLength, 1);
  float* pMatT;
  float* pATA;
  //	double fitError = 0;
  double coef;
  unsigned char* pdirImg = dirImg_.data;
  /*If the first pixel in this chain is horizontal,
   *then we try to find a horizontal line, y=ax+b;*/
  if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal) {
    /*Build the new system,and solve it using least square regression: mat * [a,b]^T = vec
     * [x0',1]         [y0']
     * [x1',1] [a]     [y1']
     *    .    [b]  =   .
     * [xn',1]         [yn']*/
    pMatT = matT.ptr<float>();  // matT = [x0', x1', ... xn'; 1,1,...,1]
    for (int i = 0; i < newLength; i++) {
      const unsigned int currentX = xCors[newOffsetS];
      const unsigned int currentY = yCors[newOffsetS];
      ++newOffsetS;
      *(pMatT + newLength) = 1.0f;
      *(pMatT++) = static_cast<float>(currentX);
      vec[0][i] = static_cast<float>(currentY);
    }
    /* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
    tempMatLineFit = matT * matT.t();
    tempVecLineFit = matT * vec;
    ATA = ATA + tempMatLineFit;
    ATV = ATV + tempVecLineFit;
    pATA = ATA.ptr<float>();
    const double a00 = static_cast<double>(pATA[0]);
    const double a01 = static_cast<double>(pATA[1]);
    const double a10 = static_cast<double>(pATA[2]);
    const double a11 = static_cast<double>(pATA[3]);
    const double v0 = static_cast<double>(ATV[0][0]);
    const double v1 = static_cast<double>(ATV[0][1]);
    const double denom = a00 * a11 - a01 * a10;
    coef = 1.0 / denom;
    lineEquation[0] = coef * (a11 * v0 - a01 * v1);
    lineEquation[1] = coef * (a00 * v1 - a10 * v0);
    /*compute line fit error */
    //		for(int i=0; i<length; i++){
    //			coef = double(yCors[offsetS]) - double(xCors[offsetS++]) * lineEquation[0] - lineEquation[1];
    //			fitError += coef*coef;
    //		}
    return 0;
  }
  /*If the first pixel in this chain is vertical,
   *then we try to find a vertical line, x=ay+b;*/
  if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical) {
    /*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
     * [y0',1]         [x0']
     * [y1',1] [a]     [x1']
     *    .    [b]  =   .
     * [yn',1]         [xn']*/
    //		pMatT = matT.GetData();//matT = [y0', y1', ... yn'; 1,1,...,1]
    pMatT = matT.ptr<float>();  // matT = [y0', y1', ... yn'; 1,1,...,1]
    for (int i = 0; i < newLength; i++) {
      *(pMatT + newLength) = 1.0f;
      *(pMatT++) = static_cast<float>(yCors[newOffsetS]);
      vec[0][i] = static_cast<float>(xCors[newOffsetS]);
      ++newOffsetS;
    }
    /* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
    //		matT.MultiplyWithTransposeOf(matT, tempMatLineFit);
    tempMatLineFit = matT * matT.t();
    tempVecLineFit = matT * vec;
    ATA = ATA + tempMatLineFit;
    ATV = ATV + tempVecLineFit;
    //		pATA = ATA.GetData();
    pATA = ATA.ptr<float>();
    const double b00 = static_cast<double>(pATA[0]);
    const double b01 = static_cast<double>(pATA[1]);
    const double b10 = static_cast<double>(pATA[2]);
    const double b11 = static_cast<double>(pATA[3]);
    const double bv0 = static_cast<double>(ATV[0][0]);
    const double bv1 = static_cast<double>(ATV[0][1]);
    const double bDenom = b00 * b11 - b01 * b10;
    coef = 1.0 / bDenom;
    lineEquation[0] = coef * (b11 * bv0 - b01 * bv1);
    lineEquation[1] = coef * (b00 * bv1 - b10 * bv0);
    /*compute line fit error */
    //		for(int i=0; i<length; i++){
    //			coef = double(xCors[offsetS]) - double(yCors[offsetS++]) * lineEquation[0] - lineEquation[1];
    //			fitError += coef*coef;
    //		}
  }
  return 0;
}

bool EDLineDetector::LineValidation_(unsigned int* xCors,
                                     unsigned int* yCors,
                                     unsigned int offsetS,
                                     unsigned int offsetE,
                                     std::array<double, 3>& lineEquation,
                                     float& direction) {
  if (bValidate_) {
    const unsigned int length = offsetE - offsetS;
    const int n = static_cast<int>(length);
    /*first compute the direction of line, make sure that the dark side always be the
     *left side of a line.*/
    int meanGradientX = 0, meanGradientY = 0;
    short* pdxImg = dxImg_.ptr<short>();
    short* pdyImg = dyImg_.ptr<short>();
    double dx, dy;
    std::vector<double> pointDirection;
    pointDirection.reserve(length);
    unsigned int currentOffset = offsetS;
    for (unsigned int i = 0; i < length; ++i) {
      const unsigned int currentX = xCors[currentOffset];
      const unsigned int currentY = yCors[currentOffset];
      ++currentOffset;
      const std::size_t index = static_cast<std::size_t>(currentY) * static_cast<std::size_t>(imageWidth) +
                                static_cast<std::size_t>(currentX);
      meanGradientX += pdxImg[index];
      meanGradientY += pdyImg[index];
      dx = static_cast<double>(pdxImg[index]);
      dy = static_cast<double>(pdyImg[index]);
      pointDirection.push_back(std::atan2(-dx, dy));
    }
    dx = std::fabs(lineEquation[1]);
    dy = std::fabs(lineEquation[0]);
    if (meanGradientX == 0 && meanGradientY == 0) {  // not possible, if happens, it must be a wrong line,
      return false;
    }
    if (meanGradientX > 0 && meanGradientY >= 0) {          // first quadrant, and positive direction of X axis.
      direction = static_cast<float>(std::atan2(-dy, dx));  // line direction is in fourth quadrant
    }
    if (meanGradientX <= 0 && meanGradientY > 0) {         // second quadrant, and positive direction of Y axis.
      direction = static_cast<float>(std::atan2(dy, dx));  // line direction is in first quadrant
    }
    if (meanGradientX < 0 && meanGradientY <= 0) {          // third quadrant, and negative direction of X axis.
      direction = static_cast<float>(std::atan2(dy, -dx));  // line direction is in second quadrant
    }
    if (meanGradientX >= 0 && meanGradientY < 0) {           // fourth quadrant, and negative direction of Y axis.
      direction = static_cast<float>(std::atan2(-dy, -dx));  // line direction is in third quadrant
    }
    /*then check whether the line is on the border of the image. We don't keep the border line.*/
    if (std::fabs(direction) < 0.15f || CV_PI - std::fabs(direction) < 0.15) {  // Horizontal line
      if (std::fabs(lineEquation[2]) < 10.0 ||
          std::fabs(static_cast<double>(imageHeight) - std::fabs(lineEquation[2])) <
              10.0) {  // upper border or lower border
        return false;
      }
    }
    if (std::fabs(std::fabs(direction) - CV_PI * 0.5) < 0.15) {  // Vertical line
      if (std::fabs(lineEquation[2]) < 10.0 || std::fabs(static_cast<double>(imageWidth) - std::fabs(lineEquation[2])) <
                                                   10.0) {  // left border or right border
        return false;
      }
    }
    // count the aligned points on the line which have the same direction as the line.
    double disDirection;
    int k = 0;
    for (unsigned int i = 0; i < length; ++i) {
      disDirection = std::fabs(static_cast<double>(direction) - pointDirection[i]);
      if (std::fabs(2 * CV_PI - disDirection) < 0.392699 ||
          disDirection < 0.392699) {  // same direction, pi/8 = 0.392699081698724
        k++;
      }
    }
    // now compute NFA(Number of False Alarms)
    double ret = nfa(n, k, 0.125, logNT_);

    return (ret > 0);  // 0 corresponds to 1 mean false alarm
  } else {
    return true;
  }
}

int EDLineDetector::EDline(cv::Mat& image) { return EDline(image, lines_); }
