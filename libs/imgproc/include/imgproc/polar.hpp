
#pragma once

#include <opencv2/core/core.hpp>
#include <utility/range.hpp>


namespace lsfm {

constexpr double DEG2RAD = CV_PI / 180;
constexpr double RAD2DEG = 180 / CV_PI;

template <class GT, class FT>
struct Polar {
  typedef GT grad_type;
  typedef FT float_type;

  //! Get range (-PI,PI)
  inline static const Range<FT> range() { return Range<FT>(static_cast<FT>(-CV_PI), static_cast<FT>(CV_PI)); }

  static inline void phase(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& phase) {
    phase.create(dx.rows, dy.cols, cv::DataType<FT>::type);
    FT* pphase = phase.ptr<FT>();

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        pphase[j] = std::atan2(static_cast<FT>(pdy[j]), static_cast<FT>(pdx[j]));
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pphase += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pphase[j] = std::atan2(static_cast<FT>(pdy[j]), static_cast<FT>(pdx[j]));
        }
      }
    }
  }

  static inline void magnitude(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag) {
    mag.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    FT* pmag = mag.ptr<FT>();
    FT vdx, vdy;

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        vdx = static_cast<FT>(pdx[j]);
        vdy = static_cast<FT>(pdy[j]);
        pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pmag += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          vdx = static_cast<FT>(pdx[j]);
          vdy = static_cast<FT>(pdy[j]);
          pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
        }
      }
    }
  }

  static inline void cart2Polar(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag, cv::Mat& phase) {
    mag.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    phase.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    FT* pmag = mag.ptr<FT>();
    FT* pphase = phase.ptr<FT>();
    FT vdx, vdy;

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        vdx = static_cast<FT>(pdx[j]);
        vdy = static_cast<FT>(pdy[j]);
        pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
        pphase[j] = std::atan2(vdy, vdx);
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pmag += dx.cols, pphase += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          vdx = static_cast<FT>(pdx[j]);
          vdy = static_cast<FT>(pdy[j]);
          pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
          pphase[j] = std::atan2(vdy, vdx);
        }
      }
    }
  }


  static inline void polar2Cart(const cv::Mat& mag, const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    dx.create(mag.rows, mag.cols, cv::DataType<GT>::type);
    dy.create(mag.rows, mag.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (mag.isContinuous() && phase.isContinuous()) {
      const FT* pmag = mag.ptr<FT>();
      const FT* pphase = phase.ptr<FT>();
      int size = mag.rows * mag.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(pmag[j] * std::cos(pphase[j]));
        pdy[j] = static_cast<GT>(pmag[j] * std::sin(pphase[j]));
      }
    } else {
      for (int i = 0; i != mag.rows; ++i, pdx += mag.cols, pdy += mag.cols) {
        const FT* pmag = mag.ptr<FT>(i);
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(pmag[j] * std::cos(pphase[j]));
          pdy[j] = static_cast<GT>(pmag[j] * std::sin(pphase[j]));
        }
      }
    }
  }

  static inline void polar2Cart(const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    dx.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    dy.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (phase.isContinuous()) {
      const FT* pphase = phase.ptr<FT>();
      int size = phase.rows * phase.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(std::cos(pphase[j]));
        pdy[j] = static_cast<GT>(std::sin(pphase[j]));
      }
    } else {
      for (int i = 0; i != phase.rows; ++i, pdx += phase.cols, pdy += phase.cols) {
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(std::cos(pphase[j]));
          pdy[j] = static_cast<GT>(std::sin(pphase[j]));
        }
      }
    }
  }

  static inline void polar2Cart(const cv::Mat& phase, FT scale, cv::Mat& dx, cv::Mat& dy) {
    dx.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    dy.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (phase.isContinuous()) {
      const FT* pphase = phase.ptr<FT>();
      int size = phase.rows * phase.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(std::cos(pphase[j]) * scale);
        pdy[j] = static_cast<GT>(std::sin(pphase[j]) * scale);
      }
    } else {
      for (int i = 0; i != phase.rows; ++i, pdx += phase.cols, pdy += phase.cols) {
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(std::cos(pphase[j]) * scale);
          pdy[j] = static_cast<GT>(std::sin(pphase[j]) * scale);
        }
      }
    }
  }
};

template <class GT, class FT>
struct PolarCV {
  typedef GT grad_type;
  typedef FT float_type;

  //! Get range (0,2PI)
  inline static const Range<FT> range() { return Range<FT>(0, static_cast<FT>(2 * CV_PI)); }

  static inline void phase(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& phase) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::phase(tdx, tdy, phase);
  }

  static inline void magnitude(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::magnitude(tdx, tdy, mag);
  }

  static inline void cart2Polar(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag, cv::Mat& phase) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::cartToPolar(tdx, tdy, mag, phase);
  }


  static inline void polar2Cart(const cv::Mat& mag, const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(mag, phase, tdx, tdy);

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  static inline void polar2Cart(const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(cv::Mat(), phase, tdx, tdy);

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  static inline void polar2Cart(const cv::Mat& phase, FT scale, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(cv::Mat(), phase, tdx, tdy);
    tdx *= scale;
    tdy *= scale;

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  //! convert to range (-PI,PI)
  static inline void wrap(const cv::Mat& angle, cv::Mat& output) {
    output = angle.clone();
    cv::Mat tmp = (static_cast<FT>(-CV_PI) - (static_cast<FT>(CV_PI) - angle));
    tmp.copyTo(output, (angle > static_cast<FT>(CV_PI)));
  }
};


}  // namespace lsfm
