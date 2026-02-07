//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file FeatureDescriptorLBD.hpp
/// @brief LBD (Line Band Descriptor) feature descriptor implementation.
/// Provides LBD descriptor structure and creator class adapted from Lilian Zhang's work.

#pragma once

#include <geometry/line.hpp>
#include <imgproc/interpolate.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>


namespace lsfm {

/// @brief LBD (Line Band Descriptor) structure.
/// Stores descriptor data as a cv::Mat with L2 distance computation.
/// @tparam FT Float type for descriptor elements
template <class FT>
struct FdLBD {
  FdLBD() {}
  /// @brief Construct from descriptor matrix.
  /// @param d The descriptor matrix
  FdLBD(const cv::Mat& d) : data(d) {}

  cv::Mat data;  ///< Descriptor data matrix

  /// @brief Compute L2 distance to another descriptor.
  /// @param rhs The other descriptor
  /// @return L2 norm distance
  inline FT distance(const FdLBD<FT>& rhs) const { return static_cast<FT>(cv::norm(data, rhs.data, cv::NORM_L2)); }

  /// @brief Compute L2 distance between two descriptors (static version).
  /// @param lhs First descriptor
  /// @param rhs Second descriptor
  /// @return L2 norm distance
  static inline FT distance(const FdLBD<FT>& lhs, const FdLBD<FT>& rhs) {
    return static_cast<FT>(cv::norm(lhs.data, rhs.data, cv::NORM_L2));
  }

  /// @brief Get descriptor type name.
  /// @return String "LBD"
  std::string name() const { return "LBD"; }
};


/// @brief Feature descriptor creator for the LBD (Line Band Descriptor).
/// Computes LBD descriptors for line segments by projecting image gradients
/// onto a line support region divided into bands, applying Gaussian weighting,
/// and constructing a normalized descriptor vector. Adapted from Lilian Zhang's work.
/// @tparam FT Float type for computations
/// @tparam GT Geometric type representing line segments
/// @tparam MT Matrix element type for gradient images
/// @tparam Interpolator Interpolation strategy for sub-pixel gradient access
template <class FT, class GT = LineSegment<FT>, class MT = short, class Interpolator = RoundNearestInterpolator<FT, MT>>
class FdcLBD : public Fdc<FT, GT, FdLBD<FT>> {
  cv::Mat dx, dy;          ///< Gradient images in x and y direction
  int numBand, widthBand;  ///< Number of bands and width of each band

  std::vector<FT> coefLocal, coefGlobal;  ///< Local and global Gaussian smoothing coefficients
  cv::Mat_<FT> bands;                     ///< Band accumulation matrix (8 rows x numBand cols)
  FT *pgdLBandSum, *ngdLBandSum, *pgdL2BandSum, *ngdL2BandSum, *pgdOBandSum, *ngdOBandSum, *pgdO2BandSum,
      *ngdO2BandSum;  ///< Pointers into band rows for positive/negative gradient sums

  int heightOfLSP, imageWidth, imageHeight,
      descriptorSize;  ///< Line support region height, image dimensions, and descriptor size

  FT invN2, invN3;  ///< Inverse normalization factors for boundary and interior bands

  /// @brief Initialize internal coefficients and data structures.
  /// Computes local/global Gaussian smoothing coefficients, sets up band
  /// accumulation matrix, and caches image dimensions and normalization factors.
  void init() {
    // compute local and global band smoothing coefficients
    ushort size = widthBand * 3;
    coefLocal.resize(size);
    FT u = size / static_cast<FT>(2);
    FT sigma = (widthBand * 2 + 1) / static_cast<FT>(2);
    FT invsigma2 = -1 / (2 * sigma * sigma);
    FT dis;

    for (ushort i = 0; i != size; ++i) {
      dis = i - u;
      coefLocal[i] = exp(dis * dis * invsigma2);
    }

    size = numBand * widthBand;
    coefGlobal.resize(size);
    u = size / static_cast<FT>(2);
    sigma = u;
    invsigma2 = -1 / (2 * sigma * sigma);
    for (ushort i = 0; i != size; ++i) {
      dis = i - u;
      coefGlobal[i] = exp(dis * dis * invsigma2);
    }


    // the summation of {g_dL |g_dL>0 } etc. for each band of the region;
    bands.create(8, numBand);
    pgdLBandSum = bands.template ptr<FT>();
    ngdLBandSum = pgdLBandSum + numBand;
    pgdL2BandSum = ngdLBandSum + numBand;
    ngdL2BandSum = pgdL2BandSum + numBand;
    pgdOBandSum = ngdL2BandSum + numBand;
    ngdOBandSum = pgdOBandSum + numBand;
    pgdO2BandSum = ngdOBandSum + numBand;
    ngdO2BandSum = pgdO2BandSum + numBand;

    // line support region data
    heightOfLSP = widthBand * numBand;

    imageWidth = dx.cols - 1;
    imageHeight = dx.rows - 1;
    descriptorSize =
        numBand * 8;  // each band, we compute the m( pgdL, ngdL,  pgdO, ngdO) and std( pgdL, ngdL,  pgdO, ngdO);

    invN2 = 1 / static_cast<FT>(widthBand * 2);
    invN3 = 1 / static_cast<FT>(widthBand * 3);
  }

 protected:
  /// @brief Compute LBD descriptor for a single line segment into a raw buffer.
  /// Projects image gradients onto the line support region, accumulates
  /// band statistics with Gaussian weighting, and normalizes the result.
  /// @param line The line segment to compute the descriptor for
  /// @param dst Pointer to output buffer (must hold at least descriptorSize elements)
  virtual void create(const GT& line, FT* dst) {
    // the summation of {g_dL |g_dL>0 } etc. for each row of the region;
    FT pgdLRowSum, ngdLRowSum, pgdL2RowSum, ngdL2RowSum, pgdORowSum, ngdORowSum, pgdO2RowSum, ngdO2RowSum;

    // store the gradient projection of pixels in support region along dL and dO vector
    FT gDL, gDO, coefInGaussion;
    ;

    int bandID;

    int lengthOfLSP = static_cast<int>(line.length());
    // get current line direction
    Vec<FT, 2> dL = line.direction();

    // exact coordinates
    Vec2<FT> sCor0, sCor, d;
    // convert upper left pos in LSR coords to local coords (rotate + translate)
    sCor0 = line.line2world(Vec2<FT>(-static_cast<FT>(lengthOfLSP - 1) / 2, -static_cast<FT>(heightOfLSP - 1) / 2),
                            line.centerPoint());

    bands.setTo(0);

    for (int hID = 0; hID < heightOfLSP; ++hID) {
      sCor = sCor0;

      pgdLRowSum = 0;
      ngdLRowSum = 0;
      pgdORowSum = 0;
      ngdORowSum = 0;

      for (int wID = 0; wID < lengthOfLSP; ++wID) {
        // To achieve rotation invariance, each simple gradient is rotated aligned with
        // the line direction and clockwise orthogonal direction.
        d.x() = Interpolator::get(dx, sCor);
        d.y() = Interpolator::get(dy, sCor);

        gDL = line.project(d);        // dx * dL[0] + dy * dL[1];
        gDO = line.normalProject(d);  // dx * dO[0] + dy * dO[1];

        if (gDL > 0)
          pgdLRowSum += gDL;
        else
          ngdLRowSum -= gDL;

        if (gDO > 0)
          pgdORowSum += gDO;
        else
          ngdORowSum -= gDO;

        // next pixel in line dir
        sCor.x() += dL[0];
        sCor.y() += dL[1];
      }
      // next row
      sCor0.x() -= dL[1];
      sCor0.y() += dL[0];

      // compute band
      coefInGaussion = coefGlobal[hID];
      pgdLRowSum = coefInGaussion * pgdLRowSum;
      ngdLRowSum = coefInGaussion * ngdLRowSum;
      pgdL2RowSum = pgdLRowSum * pgdLRowSum;
      ngdL2RowSum = ngdLRowSum * ngdLRowSum;
      pgdORowSum = coefInGaussion * pgdORowSum;
      ngdORowSum = coefInGaussion * ngdORowSum;
      pgdO2RowSum = pgdORowSum * pgdORowSum;
      ngdO2RowSum = ngdORowSum * ngdORowSum;

      // compute {g_dL |g_dL>0 }, {g_dL |g_dL<0 },
      //{g_dO |g_dO>0 }, {g_dO |g_dO<0 } of each band in the line support region
      // first, current row belong to current band;
      bandID = hID / widthBand;
      coefInGaussion = coefLocal[hID % widthBand + widthBand];
      pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
      ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
      pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
      ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
      pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
      ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
      pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
      ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;

      // In order to reduce boundary effect along the line gradient direction,
      // a row's gradient will contribute not only to its current band, but also
      // to its nearest upper and down band with gaussCoefL_.
      bandID--;
      // the band above the current band
      if (bandID >= 0) {
        coefInGaussion = coefLocal[hID % widthBand + 2 * widthBand];
        pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
        ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
        pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
        ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
        pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
        ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
        pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
        ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;
      }

      bandID = bandID + 2;
      // the band below the current band
      if (bandID < numBand) {
        coefInGaussion = coefLocal[hID % widthBand];
        pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
        ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
        pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
        ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
        pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
        ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
        pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
        ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;
      }
    }

    // construct line descriptor
    FT* desVec = dst;
    int desID;
    // Note that the first and last bands only have (lengthOfLSP * widthBand * 2.0) pixels which are counted.
    FT invN, temp;
    for (bandID = 0; bandID < numBand; ++bandID) {
      if (bandID == 0 || bandID == numBand - 1)
        invN = invN2;
      else
        invN = invN3;

      desID = bandID * 8;
      temp = pgdLBandSum[bandID] * invN;
      desVec[desID] = temp;                                                 // mean value of pgdL;
      desVec[desID + 4] = sqrt(pgdL2BandSum[bandID] * invN - temp * temp);  // std value of pgdL;

      temp = ngdLBandSum[bandID] * invN;
      desVec[desID + 1] = temp;                                             // mean value of ngdL;
      desVec[desID + 5] = sqrt(ngdL2BandSum[bandID] * invN - temp * temp);  // std value of ngdL;

      temp = pgdOBandSum[bandID] * invN;
      desVec[desID + 2] = temp;                                             // mean value of pgdO;
      desVec[desID + 6] = sqrt(pgdO2BandSum[bandID] * invN - temp * temp);  // std value of pgdO;

      temp = ngdOBandSum[bandID] * invN;
      desVec[desID + 3] = temp;                                             // mean value of ngdO;
      desVec[desID + 7] = sqrt(ngdO2BandSum[bandID] * invN - temp * temp);  // std value of ngdO;
    }

    // normalize;
    FT tempM, tempS;
    desVec = dst;
    for (ushort i = 0; i < numBand; ++i) {
      tempM = (*desVec) * *(desVec++) + (*desVec) * *(desVec++) + (*desVec) * *(desVec++) + (*desVec) * *(desVec++);
      tempS = (*desVec) * *(desVec++) + (*desVec) * *(desVec++) + (*desVec) * *(desVec++) + (*desVec) * *(desVec++);
    }
    tempM = 1 / sqrt(tempM);
    tempS = 1 / sqrt(tempS);
    desVec = dst;
    for (ushort i = 0; i < numBand; ++i) {
      (*desVec) = *(desVec++) * tempM;
      (*desVec) = *(desVec++) * tempM;
      (*desVec) = *(desVec++) * tempM;
      (*desVec) = *(desVec++) * tempM;
      (*desVec) = *(desVec++) * tempS;
      (*desVec) = *(desVec++) * tempS;
      (*desVec) = *(desVec++) * tempS;
      (*desVec) = *(desVec)*tempS;
    }

    cv::Mat dstMat(1, descriptorSize, cv::DataType<FT>::type, dst);
    // In order to reduce the influence of non-linear illumination,
    // a threshold is used to limit the value of element in the unit feature
    // vector no larger than this threshold. In Z.Wang's work, a value of 0.4 is found
    // empirically to be a proper threshold.
    dstMat.setTo(0.4, dstMat > 0.4);

    // re-normalize desVec;
    dstMat *= 1.0 / static_cast<FT>(cv::norm(dstMat, cv::NORM_L2));
  }

 public:
  typedef typename Fdc<FT, GT, FdLBD<FT>>::Ptr FdcPtr;           ///< Shared pointer to base descriptor creator
  typedef typename FdcObj<FT, GT, FdLBD<FT>>::Ptr CustomFdcPtr;  ///< Shared pointer to object-based descriptor creator
  typedef typename FdcMat<FT, GT>::Ptr SimpleFdcPtr;             ///< Shared pointer to matrix-based descriptor creator
  typedef FdLBD<FT> descriptor_type;                             ///< The descriptor type produced by this creator

  /// @brief Construct from gradient images.
  /// @param dxImg Gradient image in x direction
  /// @param dyImg Gradient image in y direction
  /// @param nBand Number of bands in the line support region (default 9)
  /// @param wBand Width of each band in pixels (default 7)
  FdcLBD(const cv::Mat& dxImg, const cv::Mat& dyImg, ushort nBand = 9, ushort wBand = 7)
      : dx(dxImg), dy(dyImg), numBand(nBand), widthBand(wBand) {
    this->options_.push_back(OptionManager::OptionEntry("num_band", nBand, "ushort", "Number of bands."));
    this->options_.push_back(OptionManager::OptionEntry("width_band", wBand, "ushort", "Width of band."));

    init();
  }

  /// @brief Construct from a named matrix map.
  /// Looks for "dx"/"gx" and "dy"/"gy" keys in the map.
  /// @param data Map of named matrices containing gradient data
  /// @param nBand Number of bands in the line support region (default 9)
  /// @param wBand Width of each band in pixels (default 7)
  FdcLBD(const MatMap& data, ushort nBand = 9, ushort wBand = 7) : numBand(nBand), widthBand(wBand) {
    this->options_.push_back(OptionManager::OptionEntry("num_band", nBand, "ushort", "Number of bands."));
    this->options_.push_back(OptionManager::OptionEntry("width_band", wBand, "ushort", "Width of band."));

    setData(data);
  }

  /// @brief Factory method to create a shared FdcLBD from gradient images.
  /// @param dxImg Gradient image in x direction
  /// @param dyImg Gradient image in y direction
  /// @param nBand Number of bands (default 9)
  /// @param wBand Width of each band (default 7)
  /// @return Shared pointer to the created FdcLBD instance
  static FdcPtr createFdc(const cv::Mat& dxImg, const cv::Mat& dyImg, ushort nBand = 9, ushort wBand = 7) {
    return FdcPtr(new FdcLBD<FT, GT, MT, Interpolator>(dxImg, dyImg, nBand, wBand));
  }

  /// @brief Factory method to create a shared FdcLBD from a named matrix map.
  /// @param data Map of named matrices containing gradient data
  /// @param nBand Number of bands (default 9)
  /// @param wBand Width of each band (default 7)
  /// @return Shared pointer to the created FdcLBD instance
  static FdcPtr createFdc(const MatMap& data, ushort nBand = 9, ushort wBand = 7) {
    return FdcPtr(new FdcLBD<FT, GT, MT, Interpolator>(data, nBand, wBand));
  }

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, FdLBD<FT>>::create;

  /// @brief Create a single LBD descriptor from a geometric object.
  /// @param input The geometric object (line segment) to describe
  /// @param dst Output descriptor object
  virtual void create(const GT& input, FdLBD<FT>& dst) {
    if (dst.data.empty() || dst.data.cols != descriptorSize) dst.data.create(1, descriptorSize, cv::DataType<FT>::type);
    this->create(input, dst.data.template ptr<FT>());
  }

  /// @brief Create a single LBD descriptor as a cv::Mat from a geometric object.
  /// @param input The geometric object (line segment) to describe
  /// @param dst Output matrix (1 x descriptorSize)
  virtual void create(const GT& input, cv::Mat& dst) {
    if (dst.empty() || dst.cols != descriptorSize) dst.create(1, descriptorSize, cv::DataType<FT>::type);
    this->create(input, dst.template ptr<FT>());
  }

  /// @brief Get the size of a single descriptor.
  /// @return Number of elements (columns in cv::Mat) per descriptor
  virtual size_t size() const { return static_cast<size_t>(descriptorSize); }

  /// @brief Set internal gradient data after construction.
  /// Accepts "dx"/"gx" and "dy"/"gy" keys from the matrix map and re-initializes.
  /// @param data Map of named matrices containing gradient data
  virtual void setData(const MatMap& data) {
    bool doInit = false;

    MatMap::const_iterator f = data.find("dx");
    if (f != data.end()) {
      dx = f->second;
      doInit = true;
    } else {
      f = data.find("gx");
      if (f != data.end()) {
        dx = f->second;
        doInit = true;
      }
    }

    f = data.find("dy");
    if (f != data.end()) {
      dy = f->second;
      doInit = true;
    } else {
      f = data.find("gy");
      if (f != data.end()) {
        dy = f->second;
        doInit = true;
      }
    }

    if (doInit) init();
  }

 protected:
  /// @brief Apply a runtime option change.
  /// Supports "num_band" (1-100) and "width_band" (1-100).
  /// @param name Option name
  /// @param value New option value
  void setOptionImpl(const std::string& name, double value) {
    if (name == "num_band") {
      if (value >= 1 && value <= 100) {
        numBand = static_cast<ushort>(value);
        this->options_[0].value = numBand;
        init();
      }
    } else if (name == "width_band") {
      if (value >= 1 && value <= 100) {
        widthBand = static_cast<ushort>(value);
        this->options_[1].value = widthBand;
        init();
      }
    }
  }
};
}  // namespace lsfm
