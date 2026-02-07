/**********************************************************************\

  SUSAN Version 2l by Stephen Smith
  Oxford Centre for Functional Magnetic Resonance Imaging of the Brain,
  Department of Clinical Neurology, Oxford University, Oxford, UK
  (Previously in Computer Vision and Image Processing Group - now
  Computer Vision and Electro Optics Group - DERA Chertsey, UK)
  Email:    steve@fmrib.ox.ac.uk
  WWW:      http://www.fmrib.ox.ac.uk/~steve

  (C) Crown Copyright (1995-1999), Defence Evaluation and Research Agency,
  Farnborough, Hampshire, GU14 6TD, UK
  DERA WWW site:
  http://www.dera.gov.uk/
  DERA Computer Vision and Electro Optics Group WWW site:
  http://www.dera.gov.uk/imageprocessing/dera/group_home.html
  DERA Computer Vision and Electro Optics Group point of contact:
  Dr. John Savage, jtsavage@dera.gov.uk, +44 1344 633203

  A UK patent has been granted: "Method for digitally processing
  images to determine the position of edges and/or corners therein for
  guidance of unmanned vehicle", UK Patent 2272285. Proprietor:
  Secretary of State for Defence, UK. 15 January 1997

  This code is issued for research purposes only and remains the
  property of the UK Secretary of State for Defence. This code must
  not be passed on without this header information being kept
  intact. This code must not be sold.

  Modified 2016-2026 by Benjamin Wassermann:
    - Ported from C to C++17 templates
    - Integrated with OpenCV cv::Mat
    - Added template type parameters for gradient/magnitude types
    - Extracted as standalone core algorithm

\**********************************************************************/

/// @file susan_core.hpp
/// @brief Core SUSAN edge detection algorithm implementation.
///
/// Standalone SUSAN (Smallest Univalue Segment Assimilating Nucleus) edge
/// detection without framework dependencies. Only requires OpenCV for
/// cv::Mat.
///
/// @see Smith, S.M. and Brady, J.M., "SUSAN - A New Approach to Low Level
///      Image Processing", Int. Journal of Computer Vision, 23(1),
///      pp. 45-78, May 1997.

#pragma once

#include <opencv2/core/core.hpp>

#include <cmath>
#include <type_traits>

namespace susan_impl {

/// @brief Safe type cast with compile-time type checking.
///
/// Returns the input value unchanged if InT and OutT are the same type,
/// otherwise performs a static_cast.
/// @tparam OutT Target output type.
/// @tparam InT Input type (deduced).
/// @param[in] in Input value to cast.
/// @return Value cast to OutT.
template <typename OutT, typename InT>
constexpr inline OutT auto_cast(const InT& in) {
  if constexpr (std::is_same<InT, OutT>::value) {
    return in;
  } else {
    return static_cast<OutT>(in);
  }
}

/// @brief Core SUSAN edge detection algorithm.
///
/// Provides static methods for SUSAN edge detection using either a
/// 37-pixel circular mask or a small 3x3 kernel. Includes brightness
/// look-up table (LUT) initialization.
/// @tparam GT Gradient component type (e.g., short).
/// @tparam MT Magnitude type (e.g., short).
template <class GT, class MT>
struct SusanCore {
  /// @brief Initialize the brightness look-up table.
  ///
  /// Computes e^(-x^6) for all possible brightness differences [-256, 256]
  /// and stores scaled values [0, 100] in the LUT for fast lookup.
  /// @param[out] bp LUT buffer of at least 516 bytes. The function writes
  ///             to bp[258 + i] for i in [-256, 256].
  /// @param[in] bt Brightness threshold controlling edge sensitivity.
  static inline void init_lut(uchar* bp, MT bt) {
    uchar* lut = bp + 258;
    for (int i = -256; i < 257; ++i) {
      double temp = static_cast<double>(i) / static_cast<double>(bt);
      temp = temp * temp * temp * temp * temp * temp;
      lut[i] = static_cast<uchar>(100 * std::exp(-temp));
    }
  }

  /// @brief SUSAN edge detection using the 3x3 kernel.
  ///
  /// Faster variant with a smaller mask for fine detail. May produce
  /// more noise than the full 37-pixel mask.
  /// @param[in] img Input grayscale image (CV_8UC1).
  /// @param[out] mag Output magnitude image.
  /// @param[out] dx Output X-gradient component image.
  /// @param[out] dy Output Y-gradient component image.
  /// @param[in] bp Brightness LUT (must be initialized via init_lut).
  /// @param[in] max_no Maximum response value for normalization.
  static inline void edges_small(
      const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& dx, cv::Mat_<GT>& dy, const uchar* bp, MT max_no) {
    int x_size = img.cols, y_size = img.rows;
    const uchar* in = img.ptr<uchar>();
    MT* r = &mag(0);
    GT* pdx = &dx(0);
    GT* pdy = &dy(0);

    int i, j, pos;
    MT m, n;
    GT x, y, w = 0;
    uchar c, do_symmetry;
    const uchar* p{nullptr};
    const uchar* cp{nullptr};
    const uchar* lut = bp + 256;

    MT scaled_max_no = static_cast<MT>(max_no * 0.277);

    for (i = 1; i < y_size - 1; ++i)
      for (j = 1; j < x_size - 1; ++j) {
        n = 100;
        pos = i * x_size + j;
        p = in + (i - 1) * x_size + j - 1;
        cp = lut + in[pos];

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 2;

        n += auto_cast<MT>(*(cp - *p));
        p += 2;
        n += *(cp - *p);
        p += x_size - 2;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));

        if (n <= scaled_max_no) r[pos] = scaled_max_no - n;
      }

    for (i = 2; i < y_size - 2; ++i)
      for (j = 2; j < x_size - 2; ++j) {
        pos = i * x_size + j;
        if (r[pos] > 0) {
          m = r[pos];
          n = scaled_max_no - m;
          cp = lut + in[pos];

          if (n > 250) {
            p = in + (i - 1) * x_size + j - 1;
            x = 0;
            y = 0;

            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            y -= auto_cast<GT>(c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y -= auto_cast<GT>(c);
            p += x_size - 2;

            c = *(cp - *p);
            x -= auto_cast<GT>(c);
            p += 2;
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            p += x_size - 2;

            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y += static_cast<GT>(auto_cast<GT>(c));
            c = *(cp - *p++);
            y += static_cast<GT>(auto_cast<MT>(c));
            c = *(cp - *p);
            x += static_cast<GT>(auto_cast<MT>(c));
            y += static_cast<GT>(auto_cast<MT>(c));

            if ((x * x + y * y) > (0.16 * n * n)) {
              do_symmetry = 0;
              float z = (x == 0) ? 1000000.0f : static_cast<float>(y) / static_cast<float>(x);
              if (z < 0) {
                z = -z;
                w = -1;
              } else
                w = 1;
              if (z < 0.25) { /* vert_edge */
                pdy[pos] = 0;
                pdx[pos] = 1;
              } else {
                if (z > 4.0) { /* hor_edge */
                  pdy[pos] = 1;
                  pdx[pos] = 0;
                } else { /* diag_edge */
                  if (w > 0) {
                    pdy[pos] = 1;
                    pdx[pos] = 1;
                  } else {
                    pdy[pos] = -1;
                    pdx[pos] = 1;
                  }
                }
              }

            } else
              do_symmetry = 1;
          } else
            do_symmetry = 1;

          if (do_symmetry == 1) {
            p = in + (i - 1) * x_size + j - 1;
            x = 0;
            y = 0;

            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(c);
            c = *(cp - *p++);
            y += auto_cast<GT>(c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(c);
            p += x_size - 2;

            c = *(cp - *p);
            x += auto_cast<GT>(c);
            p += 2;
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            p += x_size - 2;

            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(c);
            c = *(cp - *p++);
            y += auto_cast<GT>(c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(c);

            float z = (y == 0) ? 1000000.0f : static_cast<float>(x) / static_cast<float>(y);
            if (z < 0.25) { /* vertical */
              pdy[pos] = 0;
              pdx[pos] = 1;
            } else {
              if (z > 4.0) { /* horizontal */
                pdy[pos] = 1;
                pdx[pos] = 0;
              } else { /* diagonal */
                if (w > 0) {
                  pdy[pos] = -1;
                  pdx[pos] = 1;
                } else {
                  pdy[pos] = 1;
                  pdx[pos] = 1;
                }
              }
            }
          }
        }
      }
  }

  /// @brief SUSAN edge detection using the 37-pixel circular mask.
  ///
  /// Standard SUSAN edge detection with the full circular mask for
  /// robust edge detection with good noise immunity.
  /// @param[in] img Input grayscale image (CV_8UC1).
  /// @param[out] mag Output magnitude image.
  /// @param[out] dx Output X-gradient component image.
  /// @param[out] dy Output Y-gradient component image.
  /// @param[in] bp Brightness LUT (must be initialized via init_lut).
  /// @param[in] max_no Maximum response value for normalization.
  static inline void edges(
      const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& dx, cv::Mat_<GT>& dy, const uchar* bp, MT max_no) {
    int x_size = img.cols, y_size = img.rows;
    const uchar* in = img.ptr<uchar>();
    MT* r = &mag(0);
    GT* pdx = &dx(0);
    GT* pdy = &dy(0);

    int i, j, pos;
    MT m, n;
    GT x, y, w = 0;
    uchar c, do_symmetry;
    const uchar* p;
    const uchar* cp;
    const uchar* lut = bp + 256;


    for (i = 3; i < y_size - 3; ++i)
      for (j = 3; j < x_size - 3; ++j) {
        pos = i * x_size + j;
        n = 100;
        p = in + (i - 3) * x_size + j - 1;
        cp = lut + in[pos];

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 3;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 5;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 6;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += 2;
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 6;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 5;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));
        p += x_size - 3;

        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p++));
        n += auto_cast<MT>(*(cp - *p));

        if (n <= max_no) r[pos] = max_no - n;
      }

    for (i = 4; i < y_size - 4; ++i)
      for (j = 4; j < x_size - 4; ++j) {
        pos = i * x_size + j;
        if (r[pos] > 0) {
          m = r[pos];
          n = max_no - m;
          cp = lut + in[pos];

          if (n > 600) {
            p = in + (i - 3) * x_size + j - 1;
            x = 0;
            y = 0;

            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y -= auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            y -= auto_cast<GT>(3 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y -= auto_cast<GT>(3 * c);
            p += x_size - 3;

            c = *(cp - *p++);
            x -= auto_cast<GT>(2 * c);
            y -= auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y -= auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            y -= auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y -= auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(2 * c);
            y -= auto_cast<GT>(2 * c);
            p += x_size - 5;

            c = *(cp - *p++);
            x -= auto_cast<GT>(3 * c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(2 * c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(2 * c);
            y -= auto_cast<GT>(c);
            c = *(cp - *p);
            x += auto_cast<GT>(3 * c);
            y -= auto_cast<GT>(c);
            p += x_size - 6;

            c = *(cp - *p++);
            x -= auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x -= auto_cast<GT>(c);
            p += 2;
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(3 * c);
            p += x_size - 6;

            c = *(cp - *p++);
            x -= auto_cast<GT>(3 * c);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(2 * c);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y += c;
            c = *(cp - *p++);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(2 * c);
            y += auto_cast<GT>(c);
            c = *(cp - *p);
            x += auto_cast<GT>(3 * c);
            y += auto_cast<GT>(c);
            p += x_size - 5;

            c = *(cp - *p++);
            x -= auto_cast<GT>(2 * c);
            y += auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y += auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(2 * c);
            y += auto_cast<GT>(2 * c);
            p += x_size - 3;

            c = *(cp - *p++);
            x -= auto_cast<GT>(c);
            y += auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(3 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(3 * c);

            if ((x * x + y * y) > (0.81 * n * n)) {
              do_symmetry = 0;
              float z = (x == 0) ? 1000000.0f : static_cast<float>(y) / static_cast<float>(x);
              if (z < 0) {
                z = -z;
                w = -1;
              } else
                w = 1;
              if (z < 0.25) { /* vert_edge */
                pdy[pos] = 0;
                pdx[pos] = 1;
              } else {
                if (z > 4.0) { /* hor_edge */
                  pdy[pos] = 1;
                  pdx[pos] = 0;
                } else { /* diag_edge */
                  if (w > 0) {
                    pdy[pos] = 1;
                    pdx[pos] = 1;
                  } else {
                    pdy[pos] = -1;
                    pdx[pos] = 1;
                  }
                }
              }

            } else
              do_symmetry = 1;
          } else
            do_symmetry = 1;

          if (do_symmetry == 1) {
            p = in + (i - 3) * x_size + j - 1;
            x = 0;
            y = 0;
            w = 0;

            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(9 * c);
            w += auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(9 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(9 * c);
            w -= auto_cast<GT>(3 * c);
            p += x_size - 3;

            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(4 * c);
            w += auto_cast<GT>(4 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(4 * c);
            w += auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(4 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(4 * c);
            w -= auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(4 * c);
            w -= auto_cast<GT>(4 * c);
            p += x_size - 5;

            c = *(cp - *p++);
            x += auto_cast<GT>(9 * c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(c);
            c = *(cp - *p++);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(9 * c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(3 * c);
            p += x_size - 6;

            c = *(cp - *p++);
            x += auto_cast<GT>(9 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            p += 2;
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(9 * c);
            p += x_size - 6;

            c = *(cp - *p++);
            x += auto_cast<GT>(9 * c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w -= auto_cast<GT>(c);
            c = *(cp - *p++);
            y += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(c);
            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(9 * c);
            y += auto_cast<GT>(c);
            w += auto_cast<GT>(3 * c);
            p += x_size - 5;

            c = *(cp - *p++);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(4 * c);
            w -= auto_cast<GT>(4 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(4 * c);
            w -= auto_cast<GT>(2 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(4 * c);
            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(4 * c);
            w += auto_cast<GT>(2 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(4 * c);
            y += auto_cast<GT>(4 * c);
            w += auto_cast<GT>(4 * c);
            p += x_size - 3;

            c = *(cp - *p++);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(9 * c);
            w -= auto_cast<GT>(3 * c);
            c = *(cp - *p++);
            y += auto_cast<GT>(9 * c);
            c = *(cp - *p);
            x += auto_cast<GT>(c);
            y += auto_cast<GT>(9 * c);
            w += auto_cast<GT>(3 * c);

            float z = (y == 0) ? 1000000.0f : static_cast<float>(x) / static_cast<float>(y);
            if (z < 0.25) { /* vertical */
              pdy[pos] = 0;
              pdx[pos] = 1;
            } else {
              if (z > 4.0) { /* horizontal */
                pdy[pos] = 1;
                pdx[pos] = 0;
              } else { /* diagonal */
                if (w > 0) {
                  pdy[pos] = -1;
                  pdx[pos] = 1;
                } else {
                  pdy[pos] = 1;
                  pdx[pos] = 1;
                }
              }
            }
          }
        }
      }
  }
};

}  // namespace susan_impl
