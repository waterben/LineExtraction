/// @file GenericDescriptor.hpp
/// @brief Generic descriptor creation helpers and classes.
/// Provides template-based descriptor creators for various image properties like intensity,
/// gradients, and combined features, with support for rotation alignment and interpolation.

// C by Benjamin Wassermann
//


#pragma once

#include <geometry/line.hpp>
#include <imgproc/mean.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>


namespace lsfm {

/// @brief Rotation alignment helper for line-based descriptors.
/// Transforms spatial coordinates to line-aligned coordinate system (along and perpendicular to line).
/// @tparam FT Float type
template <class FT>
struct RotationAlign {
  /// @brief Transform a point to line-aligned coordinates.
  /// @param line The reference line
  /// @param p The point to transform
  /// @param ret Output point in line-aligned coordinates
  static inline void apply(const Line<FT>& line, const Vec2<FT>& p, Vec2<FT>& ret) {
    set(ret, line.project(p), line.normalProject(p));
  }

  /// @brief Transform a point and return result.
  /// @param line The reference line
  /// @param p The point to transform
  /// @return Point in line-aligned coordinates
  static inline Vec2<FT> apply(const Line<FT>& line, const Vec2<FT>& p) {
    return Vec2<FT>(line.project(p), line.normalProject(p));
  }
};

/// @brief No-op alignment helper (returns coordinates unchanged).
/// Used when rotation alignment is not needed.
/// @tparam FT Float type
template <class FT>
struct NoAlign {
  /// @brief Return input point unchanged.
  /// @param line The reference line (unused)
  /// @param p The point to transform
  /// @param ret Output point (same as input)
  static inline void apply(const Line<FT>& line, const Vec2<FT>& p, Vec2<FT>& ret) {
    static_cast<void>(line);
    ret = p;
  }

  /// @brief Return input point unchanged.
  /// @param line The reference line (unused)
  /// @param p The point to transform
  /// @return Same as input point
  static inline Vec2<FT> apply(const Line<FT>& line, const Vec2<FT>& p) {
    static_cast<void>(line);
    return p;
  }
};

/// @brief Generic feature descriptor with fixed dimensionality.
/// Stores descriptor values in fixed-size array and computes L2 distances.
/// @tparam FT Float type for descriptor elements
/// @tparam cn Number of descriptor components (compile-time constant)
template <class FT, int cn>
struct GenericDescritpor {
  GenericDescritpor() {}
  /// @brief Construct from pointer to descriptor data.
  /// @param d Pointer to cn elements
  GenericDescritpor(const FT* d) : data() { memcopy(data, d, sizeof(FT) * cn); }

  FT data[static_cast<size_t>(cn)];  ///< Descriptor data array

  /// @brief Compute L2 distance to another descriptor.
  /// @param rhs The other descriptor
  /// @return L2 norm distance
  inline FT distance(const GenericDescritpor<FT, cn>& rhs) const {
    return static_cast<FT>(norm(cv::_InputArray(data, static_cast<int>(cn)),
                                cv::_InputArray(rhs.data, static_cast<int>(cn)), cv::NORM_L2));
  }

  /// @brief Compute L2 distance between two descriptors (static version).
  /// @param lhs First descriptor
  /// @param rhs Second descriptor
  /// @return L2 norm distance
  static inline FT distance(const GenericDescritpor<FT, cn>& lhs, const GenericDescritpor<FT, cn>& rhs) {
    return static_cast<FT>(norm(cv::_InputArray(lhs.ata, cn), cv::_InputArray(rhs.data, cn), cv::NORM_L2));
  }

  /// @brief Get descriptor size.
  /// @return Number of descriptor components
  static inline int size() { return cn; }

  /// @brief Get descriptor type name.
  /// @return String "GENERIC"
  std::string name() const { return "GENERIC"; }
};


// Creator Helper for intensity images using interpolator
template <class FT, uint size = 3, uint step = 2, class Interpolator = FastRoundNearestInterpolator<FT, uchar>>
struct GchImgInterpolate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 2;


  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& img = data[0];
    // get line length (floor)
    uint length = static_cast<uint>(line.length());
    if (length < 1) length = 1;
    // get current line direction
    cv::Point_<FT> dL = line.direction();
    cv::Point_<FT> dN(-dL.y, dL.x);
    dN *= step * stepDir;
    dL *= lstep;

    // coordinates
    cv::Point_<FT> sCor0, sCor;

    FT sum, sum2, val;
    // convert upper left pos in line coords to local coords (rotate + translate)
    sCor0 =
        line.line2world(cv::Point_<FT>(-((static_cast<FT>(length) - 1) / 2), static_cast<FT>(beg)), line.centerPoint());
    int lineSteps = static_cast<int>(length / lstep);
    FT norm = static_cast<FT>(1.0 / (lineSteps * numBands));

    for (int i = 0; i != numBands; ++i) {
      sCor = sCor0;

      sum = 0;
      sum2 = 0;

      for (int j = 0; j < lineSteps; ++j) {
        // access value in mat
        val = Interpolator::get(img, sCor);
        // std::cout << sCor << std::endl;
        sum += val;
        sum2 += val * val;

        // next pixel in line dir
        sCor += dL;
      }
      // next row
      sCor0 += dN;

      // compute mean
      dst[0] = val = sum * norm;
      // compute variance
      dst[numBands] = std::sqrt(sum2 * norm - val * val);
      ++dst;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("img"));
    return ret;
  }
};

// Creator Helper for intensity images using interpolator
template <class FT, uint size = 3, uint step = 2, class Mean = FastMean<FT, uchar>>
struct GchImgMean {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 2;


  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& img = data[0];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);
    stepDir *= step;
    FT norm = static_cast<FT>(1.0 / numBands), variance, mean;

    for (int i = 0; i != numBands; ++i) {
      mean = Mean::process(variance, img, l, lstep);
      // compute mean
      dst[0] = mean * norm;
      // compute variance
      dst[numBands] = std::sqrt(variance * norm);
      ++dst;

      // next row
      l.translateOrtho(stepDir);
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("img"));
    return ret;
  }
};


// Creator Helper for intensity images using iterator
template <class FT, uint size = 3, uint step = 2, class MT = uchar>
struct GchImgIterate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 2;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& img = data[0];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);

    cv::Point_<FT> ps, pe;
    FT sum, sum2, val;

    for (int i = 0; i != numBands; ++i) {
      sum = 0;
      sum2 = 0;
      ps = l.startPoint();
      pe = l.endPoint();
      cv::LineIterator it(img, cv::Point(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y))),
                          cv::Point(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y))));
      for (int j = 0; j < it.count; ++j, ++it) {
        // access value in mat
        val = img.at<MT>(it.pos());
        sum += val;
        sum2 += val * val;
      }
      // next row
      l.translateOrtho(step * stepDir);

      // compute mean
      dst[0] = val = sum / (it.count * numBands);
      // compute variance
      dst[numBands] = std::sqrt(sum2 / (it.count * numBands) - val * val);
      ++dst;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("img"));
    return ret;
  }
};


// Creator Helper for gradient using interpolator
template <class FT,
          uint size = 3,
          uint step = 2,
          class Align = RotationAlign<FT>,
          class Interpolator = FastRoundNearestInterpolator<FT, short>>
struct GchGradInterpolate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 8;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    // get line length (floor)
    uint length = static_cast<uint>(line.length());
    if (length < 1) length = 1;
    // get current line direction
    cv::Point_<FT> dL = line.direction();
    cv::Point_<FT> dN(-dL.y, dL.x);
    dN *= step * stepDir;
    dL *= lstep;

    // coordinates
    cv::Point_<FT> sCor0, sCor, val;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, tmp;
    // convert upper left pos in line coords to local coords (rotate + translate)
    sCor0 =
        line.line2world(cv::Point_<FT>(-((static_cast<FT>(length) - 1) / 2), static_cast<FT>(beg)), line.centerPoint());
    int lineSteps = static_cast<int>(length / lstep);
    FT norm = static_cast<FT>(1.0 / (lineSteps * numBands));

    for (int i = 0; i != numBands; ++i) {
      sCor = sCor0;

      sumXP = sumXN = sumYP = sumYN = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = 0;

      for (int j = 0; j < lineSteps; ++j) {
        // access value in grad and do alignment
        Align::apply(line, cv::Point_<FT>(Interpolator::get(dx, sCor), Interpolator::get(dy, sCor)), val);
        if (val.x < 0) {
          sumXN -= val.x;
          sum2XN += val.x * val.x;
        } else {
          sumXP += val.x;
          sum2XP += val.x * val.x;
        }
        if (val.y < 0) {
          sumYN -= val.y;
          sum2YN += val.y * val.y;
        } else {
          sumYP += val.y;
          sum2YP += val.y * val.y;
        }

        // next pixel in line dir
        sCor += dL;
      }
      // next row
      sCor0 += dN;

      // compute mean and compute variance
      dst[0] = tmp = sumXP * norm;
      dst[4] = std::sqrt(sum2XP * norm - tmp * tmp);
      dst[1] = tmp = sumXN * norm;
      dst[5] = std::sqrt(sum2XN * norm - tmp * tmp);
      dst[2] = tmp = sumYP * norm;
      dst[6] = std::sqrt(sum2YP * norm - tmp * tmp);
      dst[3] = tmp = sumYN * norm;
      dst[7] = std::sqrt(sum2YN * norm - tmp * tmp);
      dst += 8;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    return ret;
  }
};

// Creator Helper for intensity images using interator
template <class FT, uint size = 3, uint step = 2, class Align = RotationAlign<FT>, class Mean = FastMean<FT, short>>
struct GchGradMean {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 8;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);
    stepDir *= step;
    cv::Point_<FT> p;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, tmp, norm;
    std::vector<FT> valX, valY;
    size_t n;

    for (int i = 0; i != numBands; ++i) {
      sumXP = sumXN = sumYP = sumYN = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = 0;
      Mean::process(valX, dx, l, lstep);
      Mean::process(valY, dy, l, lstep);
      n = valX.size();
      for (int j = 0; j < n; ++j) {
        // access value in grad and do alignment
        Align::apply(l, cv::Point_<FT>(valX[j], valY[j]), p);
        if (p.x < 0) {
          sumXN -= p.x;
          sum2XN += p.x * p.x;
        } else {
          sumXP += p.x;
          sum2XP += p.x * p.x;
        }
        if (p.y < 0) {
          sumYN -= p.y;
          sum2YN += p.y * p.y;
        } else {
          sumYP += p.y;
          sum2YP += p.y * p.y;
        }
      }
      // next row
      l.translateOrtho(stepDir);
      norm = static_cast<FT>(1) / (n * numBands);
      // compute mean and compute variance
      dst[0] = tmp = sumXP * norm;
      dst[4] = std::sqrt(sum2XP * norm - tmp * tmp);
      dst[1] = tmp = sumXN * norm;
      dst[5] = std::sqrt(sum2XN * norm - tmp * tmp);
      dst[2] = tmp = sumYP * norm;
      dst[6] = std::sqrt(sum2YP * norm - tmp * tmp);
      dst[3] = tmp = sumYN * norm;
      dst[7] = std::sqrt(sum2YN * norm - tmp * tmp);
      dst += 8;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    return ret;
  }
};


// Creator Helper for intensity images using interator
template <class FT, uint size = 3, uint step = 2, class Align = RotationAlign<FT>, class MT = short>
struct GchGradIterate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 8;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);

    cv::Point_<FT> ps, pe;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, tmp;

    for (int i = 0; i != numBands; ++i) {
      sumXP = sumXN = sumYP = sumYN = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = 0;
      ps = l.startPoint();
      pe = l.endPoint();
      cv::LineIterator it(dx, cv::Point(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y))),
                          cv::Point(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y))));

      for (int j = 0; j < it.count; ++j, ++it) {
        // access value in grad and do alignment
        Align::apply(l, cv::Point_<FT>(dx.at<MT>(it.pos()), dy.at<MT>(it.pos())), ps);
        if (ps.x < 0) {
          sumXN -= ps.x;
          sum2XN += ps.x * ps.x;
        } else {
          sumXP += ps.x;
          sum2XP += ps.x * ps.x;
        }
        if (ps.y < 0) {
          sumYN -= ps.y;
          sum2YN += ps.y * ps.y;
        } else {
          sumYP += ps.y;
          sum2YP += ps.y * ps.y;
        }
      }
      // next row
      l.translateOrtho(step * stepDir);

      // compute mean and compute variance
      dst[0] = tmp = sumXP / (it.count * numBands);
      dst[4] = std::sqrt(sum2XP / (it.count * numBands) - tmp * tmp);
      dst[1] = tmp = sumXN / (it.count * numBands);
      dst[5] = std::sqrt(sum2XN / (it.count * numBands) - tmp * tmp);
      dst[2] = tmp = sumYP / (it.count * numBands);
      dst[6] = std::sqrt(sum2YP / (it.count * numBands) - tmp * tmp);
      dst[3] = tmp = sumYN / (it.count * numBands);
      dst[7] = std::sqrt(sum2YN / (it.count * numBands) - tmp * tmp);
      dst += 8;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    return ret;
  }
};

// Creator Helper for gradient and image using interpolator
template <class FT,
          uint size = 3,
          uint step = 2,
          class Align = RotationAlign<FT>,
          class InterpolatorG = FastRoundNearestInterpolator<FT, short>,
          class InterpolatorI = FastRoundNearestInterpolator<FT, uchar>>
struct GchGradImgInterpolate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 10;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    const cv::Mat& img = data[2];
    // get line length (floor)
    uint length = static_cast<uint>(line.length());
    if (length < 1) length = 1;
    // get current line direction
    Vec2<FT> dL = line.direction();
    Vec2<FT> dN(-dL.y(), dL.x());
    dN *= step * stepDir;
    dL *= lstep;

    // coordinates
    Vec2<FT> sCor0, sCor, val;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, sum, sum2, tmp;
    // convert upper left pos in line coords to local coords (rotate + translate)
    sCor0 = line.line2world(Vec2<FT>(-((static_cast<FT>(length) - 1) / 2), static_cast<FT>(beg)), line.centerPoint());
    int lineSteps = static_cast<int>(length / lstep);
    FT norm = static_cast<FT>(1.0 / (lineSteps * numBands));

    for (int i = 0; i != numBands; ++i) {
      sCor = sCor0;

      sumXP = sumXN = sumYP = sumYN = sum = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = sum2 = 0;

      for (int j = 0; j < lineSteps; ++j) {
        // access value in grad and do alignment
        Align::apply(line, Vec2<FT>(InterpolatorG::get(dx, sCor), InterpolatorG::get(dy, sCor)), val);
        if (val.x() < 0) {
          sumXN -= val.x();
          sum2XN += val.x() * val.x();
        } else {
          sumXP += val.x();
          sum2XP += val.x() * val.x();
        }
        if (val.y() < 0) {
          sumYN -= val.y();
          sum2YN += val.y() * val.y();
        } else {
          sumYP += val.y();
          sum2YP += val.y() * val.y();
        }

        tmp = InterpolatorI::get(img, sCor);
        sum += tmp;
        sum2 += tmp * tmp;

        // next pixel in line dir
        sCor += dL;
      }
      // next row
      sCor0 += dN;

      // compute mean and compute variance of grad
      dst[0] = tmp = sumXP * norm;
      dst[5] = std::sqrt(sum2XP * norm - tmp * tmp);
      dst[1] = tmp = sumXN * norm;
      dst[6] = std::sqrt(sum2XN * norm - tmp * tmp);
      dst[2] = tmp = sumYP * norm;
      dst[7] = std::sqrt(sum2YP * norm - tmp * tmp);
      dst[3] = tmp = sumYN * norm;
      dst[8] = std::sqrt(sum2YN * norm - tmp * tmp);
      // compute mean and compute variance of img
      tmp = sum * norm;
      dst[4] = tmp * 2;
      dst[9] = std::sqrt(sum2 * norm - tmp * tmp) * 2;
      dst += 10;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    ret.push_back(std::string("img"));
    return ret;
  }
};

// Creator Helper for intensity images using interator
template <class FT,
          uint size = 3,
          uint step = 2,
          class Align = RotationAlign<FT>,
          class MeanG = FastMean<FT, short>,
          class MeanI = FastMean<FT, uchar>>
struct GchGradImgMean {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 8;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    const cv::Mat& img = data[2];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);
    stepDir *= step;
    cv::Point_<FT> p;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, sum, sum2, tmp, norm;
    std::vector<FT> valX, valY, valI;
    size_t n;

    for (int i = 0; i != numBands; ++i) {
      sumXP = sumXN = sumYP = sumYN = sum = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = sum2 = 0;
      MeanG::process(valX, dx, l, lstep);
      MeanG::process(valY, dy, l, lstep);
      MeanI::process(valI, img, l, lstep);
      n = valX.size();
      for (int j = 0; j < n; ++j) {
        // access value in grad and do alignment
        Align::apply(l, cv::Point_<FT>(valX[j], valY[j]), p);
        if (p.x < 0) {
          sumXN -= p.x;
          sum2XN += p.x * p.x;
        } else {
          sumXP += p.x;
          sum2XP += p.x * p.x;
        }
        if (p.y < 0) {
          sumYN -= p.y;
          sum2YN += p.y * p.y;
        } else {
          sumYP += p.y;
          sum2YP += p.y * p.y;
        }
        tmp = valI[j];
        sum += tmp;
        sum2 += tmp * tmp;
      }
      // next row
      l.translateOrtho(stepDir);
      norm = static_cast<FT>(1) / (n * numBands);
      // compute mean and compute variance
      dst[0] = tmp = sumXP * norm;
      dst[5] = std::sqrt(sum2XP * norm - tmp * tmp);
      dst[1] = tmp = sumXN * norm;
      dst[6] = std::sqrt(sum2XN * norm - tmp * tmp);
      dst[2] = tmp = sumYP * norm;
      dst[7] = std::sqrt(sum2YP * norm - tmp * tmp);
      dst[3] = tmp = sumYN * norm;
      dst[9] = std::sqrt(sum2YN * norm - tmp * tmp);
      // compute mean and compute variance of img
      tmp = sum * norm;
      dst[4] = tmp * 2;
      dst[9] = std::sqrt(sum2 * norm - tmp * tmp) * 2;
      dst += 10;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    ret.push_back(std::string("img"));
    return ret;
  }
};

// Creator Helper for intensity images using interator
template <class FT, uint size = 3, uint step = 2, class Align = RotationAlign<FT>, class MTG = short, class MTI = uchar>
struct GchGradImgIterate {
  static constexpr int numBands = size / step + (size % step ? 1 : 0);
  static constexpr int dscSize = numBands * 10;

  static void create(
      const cv::Mat* data, const LineSegment<FT>& line, FT* dst, FT beg = -1, FT stepDir = 1, FT lstep = 1) {
    const cv::Mat& dx = data[0];
    const cv::Mat& dy = data[1];
    const cv::Mat& img = data[2];
    LineSegment<FT> l = line;
    l.translateOrtho(beg);

    cv::Point_<FT> ps, pe;
    FT sumXP, sumXN, sumYP, sumYN, sum2XP, sum2XN, sum2YP, sum2YN, sum, sum2, tmp;

    for (int i = 0; i != numBands; ++i) {
      sumXP = sumXN = sumYP = sumYN = sum = 0;
      sum2XP = sum2XN = sum2YP = sum2YN = sum2 = 0;
      ps = l.startPoint();
      pe = l.endPoint();
      cv::LineIterator it(dx, cv::Point(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y))),
                          cv::Point(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y))));

      for (int j = 0; j < it.count; ++j, ++it) {
        // access value in grad and do alignment
        Align::apply(l, cv::Point_<FT>(dx.at<MTG>(it.pos()), dy.at<MTG>(it.pos())), ps);
        if (ps.x < 0) {
          sumXN -= ps.x;
          sum2XN += ps.x * ps.x;
        } else {
          sumXP += ps.x;
          sum2XP += ps.x * ps.x;
        }
        if (ps.y < 0) {
          sumYN -= ps.y;
          sum2YN += ps.y * ps.y;
        } else {
          sumYP += ps.y;
          sum2YP += ps.y * ps.y;
        }

        tmp = img.at<MTI>(it.pos());
        sum += tmp;
        sum2 += tmp * tmp;
      }
      // next row
      l.translateOrtho(step * stepDir);

      // compute mean and compute variance of grad
      dst[0] = tmp = sumXP / (it.count * numBands);
      dst[5] = std::sqrt(sum2XP / (it.count * numBands) - tmp * tmp);
      dst[1] = tmp = sumXN / (it.count * numBands);
      dst[6] = std::sqrt(sum2XN / (it.count * numBands) - tmp * tmp);
      dst[2] = tmp = sumYP / (it.count * numBands);
      dst[7] = std::sqrt(sum2YP / (it.count * numBands) - tmp * tmp);
      dst[3] = tmp = sumYN / (it.count * numBands);
      dst[8] = std::sqrt(sum2YN / (it.count * numBands) - tmp * tmp);
      // compute mean and compute variance of img
      tmp = sum / (it.count * numBands);
      dst[4] = tmp * 2;
      dst[9] = std::sqrt(sum2 / (it.count * numBands) - tmp * tmp) * 2;
      dst += 10;
    }
  }

  static std::vector<std::string> inputData() {
    std::vector<std::string> ret;
    ret.push_back(std::string("gx"));
    ret.push_back(std::string("gy"));
    ret.push_back(std::string("img"));
    return ret;
  }
};

// Generic Feature Descriptor creator for gradient
template <class FT, class GT = LineSegment<FT>, class Helper = GchImgInterpolate<FT>>
class FdcGeneric : public Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>> {
 public:
  typedef typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr FdcPtr;
  typedef typename FdcObj<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr CustomFdcPtr;
  typedef typename FdcMat<FT, GT>::Ptr SimpleFdcPtr;
  typedef GenericDescritpor<FT, Helper::dscSize> descriptor_type;

  FdcGeneric(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1)
      : data_(), pos_(pos), stepDir_(stepDir), lstep_(lstep) {
    data_.resize(Helper::inputData().size());
    this->setData(data);
  }

  static FdcPtr createFdc(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1) {
    return FdcPtr(new FdcGeneric<FT, GT, Helper>(data, pos, stepDir, lstep));
  }

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::create;


  //! create single descriptor from single geometric object
  virtual void create(const GT& input, descriptor_type& dst) {
    Helper::create(data_.data(), input, dst.data, pos_, stepDir_, lstep_);
  }

  //! create single simple descriptor from geometric object
  virtual void create(const GT& input, cv::Mat& dst) {
    if (dst.empty() || dst.cols != descriptor_type::size())
      dst.create(1, descriptor_type::size(), cv::DataType<FT>::type);
    Helper::create(data_.data(), input, dst.template ptr<FT>(), pos_, stepDir_, lstep_);
  }

  //! get size of single descriptor (cols in cv::Mat)
  virtual size_t size() const { return static_cast<size_t>(descriptor_type::size()); }

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) {
    MatMap::const_iterator f;
    auto input = Helper::inputData();
    for (size_t i = 0; i != input.size(); ++i) {
      f = data.find(input[i]);
      if (f != data.end()) data_[i] = f->second;
    }
  }

  // input
  std::vector<cv::Mat> data_;
  FT pos_, stepDir_, lstep_;

 protected:
  virtual void create(const GT& input, FT* dst) { Helper::create(data_.data(), input, dst, pos_, stepDir_, lstep_); }
};


template <class FT, class GT = LineSegment<FT>, class Helper = GchImgInterpolate<FT>>
typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr createGenericFdc(const cv::Mat& img,
                                                                                   FT pos = -1,
                                                                                   FT stepDir = 1,
                                                                                   FT lstep = 1) {
  MatMap tmp;
  tmp["img"] = img;
  return typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGeneric<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}

template <class FT, class GT = LineSegment<FT>, class Helper = GchGradInterpolate<FT>>
typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr createGenericFdc(
    const cv::Mat& gx, const cv::Mat& gy, FT pos = -1, FT stepDir = 1, FT lstep = 1) {
  MatMap tmp;
  tmp["gx"] = gx;
  tmp["gy"] = gy;
  return typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGeneric<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}

template <class FT, class GT = LineSegment<FT>, class Helper = GchGradImgInterpolate<FT>>
typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr createGenericFdc(
    const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& img, FT pos = -1, FT stepDir = 1, FT lstep = 1) {
  MatMap tmp;
  tmp["gx"] = gx;
  tmp["gy"] = gy;
  tmp["img"] = img;
  return typename Fdc<FT, GT, GenericDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGeneric<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}
}  // namespace lsfm
