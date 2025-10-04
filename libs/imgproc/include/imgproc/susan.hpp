/* {{{ Copyright etc. */

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

\**********************************************************************/

/* }}} */
/* {{{ Readme First */

/**********************************************************************\

SUSAN Version 2l
SUSAN = Smallest Univalue Segment Assimilating Nucleus

Email:    steve@fmrib.ox.ac.uk
WWW:      http://www.fmrib.ox.ac.uk/~steve

Related paper:
@article{Smith97,
author = "Smith, S.M. and Brady, J.M.",
title = "{SUSAN} - A New Approach to Low Level Image Processing",
journal = "Int. Journal of Computer Vision",
pages = "45--78",
volume = "23",
number = "1",
month = "May",
year = 1997}

To be registered for automatic (bug) updates of SUSAN, send an email.

Compile with:
gcc -O4 -o susan susan2l.c -lm

See following section for different machine information. Please
report any bugs (and fixes). There are a few optional changes that
can be made in the "defines" section which follows shortly.

Usage: type "susan" to get usage. Only PGM format files can be input
and output. Utilities such as the netpbm package and XV can be used
to convert to and from other formats. Any size of image can be
processed.

This code is written using an emacs folding mode, making moving
around the different sections very easy. This is why there are
various marks within comments and why comments are indented.


SUSAN QUICK:

This version of the SUSAN corner finder does not do all the
false-corner suppression and thus is faster and produced some false
positives, particularly on strong edges. However, because there are
less stages involving thresholds etc., the corners that are
correctly reported are usually more stable than those reported with
the full algorithm. Thus I recommend at least TRYING this algorithm
for applications where stability is important, e.g., tracking.

THRESHOLDS:

There are two thresholds which can be set at run-time. These are the
brightness threshold (t) and the distance threshold (d).

SPATIAL CONTROL: d

In SUSAN smoothing d controls the size of the Gaussian mask; its
default is 4.0. Increasing d gives more smoothing. In edge finding,
a fixed flat mask is used, either 37 pixels arranged in a "circle"
(default), or a 3 by 3 mask which gives finer detail. In corner
finding, only the larger 37 pixel mask is used; d is not
variable. In smoothing, the flat 3 by 3 mask can be used instead of
a larger Gaussian mask; this gives low smoothing and fast operation.

BRIGHTNESS CONTROL: t

In all three algorithms, t can be varied (default=20); this is the
main threshold to be varied. It determines the maximum difference in
greylevels between two pixels which allows them to be considered
part of the same "region" in the image. Thus it can be reduced to
give more edges or corners, i.e. to be more sensitive, and vice
versa. In smoothing, reducing t gives less smoothing, and vice
versa. Set t=10 for the test image available from the SUSAN web
page.

ITERATIONS:

With SUSAN smoothing, more smoothing can also be obtained by
iterating the algorithm several times. This has a different effect
from varying d or t.

FIXED MASKS:

37 pixel mask:    ooo       3 by 3 mask:  ooo
                 ooooo                    ooo
                ooooooo                   ooo
                ooooooo
                ooooooo
                 ooooo
                  ooo

CORNER ATTRIBUTES dx, dy and I
(Only read this if you are interested in the C implementation or in
using corner attributes, e.g., for corner matching)

Corners reported in the corner list have attributes associated with
them as well as positions. This is useful, for example, when
attempting to match corners from one image to another, as these
attributes can often be fairly unchanged between images. The
attributes are dx, dy and I. I is the value of image brightness at
the position of the corner. In the case of susan_corners_quick, dx
and dy are the first order derivatives (differentials) of the image
brightness in the x and y directions respectively, at the position
of the corner. In the case of normal susan corner finding, dx and dy
are scaled versions of the position of the centre of gravity of the
USAN with respect to the centre pixel (nucleus).

BRIGHTNESS FUNCTION LUT IMPLEMENTATION:
(Only read this if you are interested in the C implementation)

The SUSAN brightness function is implemented as a LUT
(Look-Up-Table) for speed. The resulting pointer-based code is a
little hard to follow, so here is a brief explanation. In
setup_brightness_lut() the LUT is setup. This mallocs enough space
for *bp_ and then repositions the pointer to the centre of the
malloced space. The SUSAN function e^-(x^6) or e^-(x^2) is
calculated and converted to a uchar in the range 0-100, for all
possible image brightness differences (including negative
ones). Thus bp_[23] is the output for a brightness difference of 23
greylevels. In the SUSAN algorithms this LUT is used as follows:

p=in + (i-3)*x_size + j - 1;
p points to the first image pixel in the circular mask surrounding
point (x,y).

cp=bp + in[i*x_size+j];
cp points to a position in the LUT corresponding to the brightness
of the centre pixel (x,y).

now for every pixel within the mask surrounding (x,y),
n+=*(cp-*p++);
the brightness difference function is found by moving the cp pointer
down by an amount equal to the value of the pixel pointed to by p,
thus subtracting the two brightness values and performing the
exponential function. This value is added to n, the running USAN
area.

in SUSAN smoothing, the variable height mask is implemented by
multiplying the above by the moving mask pointer, reset for each new
centre pixel.
tmp = *dpt++ * *(cp-brightness);

\**********************************************************************/

/* }}} */
/* {{{ Machine Information */

/**********************************************************************\

Success has been reported with the following:

MACHINE  OS         COMPILER

Sun      4.1.4      bundled C, gcc

Next

SGI      IRIX       SGI cc

DEC      Unix V3.2+

IBM RISC AIX        gcc

PC                  Borland 5.0

PC       Linux      gcc-2.6.3

PC       Win32      Visual C++ 4.0 (Console Application)

PC       Win95      Visual C++ 5.0 (Console Application)
Thanks to Niu Yongsheng <niuysbit@163.net>:
Use the FOPENB option below

PC       DOS        djgpp gnu C
Thanks to Mark Pettovello <mpettove@umdsun2.umd.umich.edu>:
Use the FOPENB option below

HP       HP-UX      bundled cc
Thanks to Brian Dixon <briand@hpcvsgen.cv.hp.com>:
in ksh:
export CCOPTS="-Aa -D_HPUX_SOURCE | -lM"
cc -O3 -o susan susan2l.c

\**********************************************************************/

/* }}} */
/* {{{ History */

/**********************************************************************\

SUSAN Version 2l, 12/2/99
Changed GNUDOS option to FOPENB.
(Thanks to Niu Yongsheng <niuysbit@163.net>.)
Took out redundant "sq=sq/2;".

SUSAN Version 2k, 19/8/98:
In corner finding:
Changed if(yy<sq) {...} else if(xx<sq) {...} to
if(yy<xx) {...} else {...}
(Thanks to adq@cim.mcgill.edu - Alain Domercq.)

SUSAN Version 2j, 22/10/97:
Fixed (mask_size>x_size) etc. tests in smoothing.
Added a couple of free() calls for cgx and cgy.
(Thanks to geoffb@ucs.ed.ac.uk - Geoff Browitt.)

SUSAN Version 2i, 21/7/97:
Added information about corner attributes.

SUSAN Version 2h, 16/12/96:
Added principle (initial enhancement) option.

SUSAN Version 2g, 2/7/96:
Minor superficial changes to code.

SUSAN Version 2f, 16/1/96:
Added GNUDOS option (now called FOPENB; see options below).

SUSAN Version 2e, 9/1/96:
Added -b option.
Fixed 1 pixel horizontal offset error for drawing edges.

SUSAN Version 2d, 27/11/95:
Fixed loading of certain PGM files in get_image (again!)

SUSAN Version 2c, 22/11/95:
Fixed loading of certain PGM files in get_image.
(Thanks to qu@San-Jose.ate.slb.com - Gongyuan Qu.)

SUSAN Version 2b, 9/11/95:
removed "z==" error in edges routines.

SUSAN Version 2a, 6/11/95:
Removed a few unnecessary variable declarations.
Added different machine information.
Changed "header" in get_image to char.

SUSAN Version 2, 1/11/95: first combined version able to take any
image sizes.

SUSAN "Versions 1", circa 1992: the various SUSAN algorithms were
developed during my doctorate within different programs and for
fixed image sizes. The algorithms themselves are virtually unaltered
between "versions 1" and the combined program, version 2.

\**********************************************************************/

/* }}} */
/* {{{ defines, includes and typedefs */

/* ********** Optional settings */

#pragma once

#include <imgproc/gradient.hpp>

namespace lsfm {

template <class GT = short, class MT = short, class DT = float, class DO = Direction<GT, DT>>
class SusanGradient : public Gradient<uchar, GT, MT, DT> {
  // Thresholds for brightness and distance
  MT bt_, max_no_;

  bool small_kernel_;
  mutable bool dir_done_;

  // LUT
  uchar bp_[516];

  cv::Mat_<MT> mag_;
  cv::Mat_<GT> dx_, dy_;
  mutable cv::Mat_<DT> dir_;


  void initLut() {
    uchar* bp = bp_ + 258;

    for (int i = -256; i < 257; ++i) {
      double temp = static_cast<double>(i) / static_cast<double>(bt_);

      temp = temp * temp * temp * temp * temp * temp;
      bp[i] = static_cast<uchar>(100 * std::exp(-temp));
    }
  }


 public:
  typedef uchar img_type;
  typedef MT mag_type;
  typedef GT grad_type;
  typedef DT dir_type;

  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<DT> DirectionRange;


  //! SUSAN Gradient Constructor
  //! PARAMETERS:
  //! bt to define the brightness threshold [0-256]?
  //! three_by_three to use the small kernel, default:false
  SusanGradient(MT bt = 20,
                bool small_kernel = false,
                MT max_no = 2650,
                uchar int_lower = std::numeric_limits<uchar>::lowest(),
                uchar int_upper = std::numeric_limits<uchar>::max())
      : Gradient<uchar, GT, MT, dir_type>(int_lower, int_upper), bt_(bt), max_no_(max_no), small_kernel_(small_kernel) {
    this->add("grad_brightness_th",
              std::bind(&SusanGradient<GT, MT, DT, DO>::brightnessTh, this, std::placeholders::_1),
              "Brightness threshold [0-256].");
    // this->add("distance_th", std::bind(&SusanGradient<GT,MT,DT,DO>::distanceTh,this,std::placeholders::_1),"Distance
    // threshold [0-256]."); this->add("form",
    // std::bind(&SusanGradient<GT,MT,DT,DO>::form,this,std::placeholders::_1),"form of the LUT [2,6].");
    this->add("grad_small_kernel", std::bind(&SusanGradient<GT, MT, DT, DO>::smallKernel, this, std::placeholders::_1),
              "Use small kernel size.");
    this->add("grad_max_no", std::bind(&SusanGradient<GT, MT, DT, DO>::maxNo, this, std::placeholders::_1),
              "Maximum number for edges (2650).");

    initLut();
  }

  Value brightnessTh(const Value& bt = Value::NAV()) {
    if (bt.type()) {
      bt_ = bt;
      initLut();
    }
    return bt_;
  }
  // Value distanceTh(const Value &dt = Value::NAV()) { if (dt.type()) dt_ = dt; return dt_;}
  // Value form(const Value &fo = Value::NAV()) { if (fo.type()) {form_ = static_cast<ushort>(fo.getInt()); initLut(); }
  // return form_;}
  Value smallKernel(const Value& sk = Value::NAV()) {
    if (sk.type()) small_kernel_ = sk;
    return small_kernel_;
  }
  Value maxNo(const Value& mn = Value::NAV()) {
    if (mn.type()) max_no_ = mn;
    return max_no_;
  }

  //! process gradient
  inline void process(const cv::Mat& img) {
    dx_.create(img.rows, img.cols);
    dx_.setTo(0);
    dy_.create(img.rows, img.cols);
    dy_.setTo(0);
    mag_.create(img.rows, img.cols);
    mag_.setTo(0);

    dir_done_ = false;

    if (small_kernel_)
      susan_edges_small(img);
    else
      susan_edges(img);
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) {
    process(img);
    directionals(gx, gy);
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  //! get x,y derivatives
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = dx_;
    gy = dy_;
  }

  //! get x derivative
  cv::Mat gx() const { return dx_; }

  //! get y derivative
  cv::Mat gy() const { return dy_; }

  //! get magnitude
  cv::Mat magnitude() const { return mag_; }

  //! test if direction is computed
  inline bool isDirectionDone() const { return dir_done_; }

  //! get direction
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(dx_, dy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return DO::range(); }

  MagnitudeRange magnitudeRange() const {
    return MagnitudeRange(0, small_kernel_ ? static_cast<int>(max_no_ * 0.277) : max_no_);
  }

  GradientRange gradientRange() const {
    GT val = small_kernel_ ? 6 * this->intensityRange().upper : 26 * this->intensityRange().upper;
    return GradientRange(-val, val);
  }


  //! get name of gradient operator
  inline std::string name() const { return "susan"; }

 private:
  void susan_edges_small(const cv::Mat& img) {
    int x_size = img.cols, y_size = img.rows;
    const uchar* in = img.ptr<uchar>();
    MT* r = &mag_(0);
    GT* dx = &dx_(0);
    GT* dy = &dy_(0);

    int i, j, pos;
    MT m, n;
    GT x, y, w = 0;
    uchar c, do_symmetry;
    const uchar* p;
    const uchar* cp;
    const uchar* bp = bp_ + 256;

    MT max_no = static_cast<MT>(max_no_ * 0.277);

    for (i = 1; i < y_size - 1; ++i)
      for (j = 1; j < x_size - 1; ++j) {
        n = 100;
        pos = i * x_size + j;
        p = in + (i - 1) * x_size + j - 1;
        cp = bp + in[pos];

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 2;

        n += *(cp - *p);
        p += 2;
        n += *(cp - *p);
        p += x_size - 2;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);

        if (n <= max_no) r[pos] = max_no - n;
      }

    for (i = 2; i < y_size - 2; ++i)
      for (j = 2; j < x_size - 2; ++j) {
        pos = i * x_size + j;
        if (r[pos] > 0) {
          m = r[pos];
          n = max_no - m;
          cp = bp + in[pos];

          if (n > 250) {
            p = in + (i - 1) * x_size + j - 1;
            x = 0;
            y = 0;

            c = *(cp - *p++);
            x -= c;
            y -= c;
            c = *(cp - *p++);
            y -= c;
            c = *(cp - *p);
            x += c;
            y -= c;
            p += x_size - 2;

            c = *(cp - *p);
            x -= c;
            p += 2;
            c = *(cp - *p);
            x += c;
            p += x_size - 2;

            c = *(cp - *p++);
            x -= c;
            y += c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p);
            x += c;
            y += c;

            if ((x * x + y * y) > (0.16 * n * n)) {
              do_symmetry = 0;
              float z = (x == 0) ? 1000000.0f : static_cast<float>(y) / static_cast<float>(x);
              if (z < 0) {
                z = -z;
                w = -1;
              } else
                w = 1;
              if (z < 0.25) { /* vert_edge */
                dy[pos] = 0;
                dx[pos] = 1;
              } else {
                if (z > 4.0) { /* hor_edge */
                  dy[pos] = 1;
                  dx[pos] = 0;
                } else { /* diag_edge */
                  if (w > 0) {
                    dy[pos] = 1;
                    dx[pos] = 1;
                  } else {
                    dy[pos] = -1;
                    dx[pos] = 1;
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
            x += c;
            y += c;
            w += c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p);
            x += c;
            y += c;
            w -= c;
            p += x_size - 2;

            c = *(cp - *p);
            x += c;
            p += 2;
            c = *(cp - *p);
            x += c;
            p += x_size - 2;

            c = *(cp - *p++);
            x += c;
            y += c;
            w -= c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p);
            x += c;
            y += c;
            w += c;

            float z = (y == 0) ? 1000000.0f : static_cast<float>(x) / static_cast<float>(y);
            if (z < 0.25) { /* vertical */
              dy[pos] = 0;
              dx[pos] = 1;
            } else {
              if (z > 4.0) { /* horizontal */
                dy[pos] = 1;
                dx[pos] = 0;
              } else { /* diagonal */
                if (w > 0) {
                  dy[pos] = -1;
                  dx[pos] = 1;
                } else {
                  dy[pos] = 1;
                  dx[pos] = 1;
                }
              }
            }
          }
        }
      }
  }

  void susan_edges(const cv::Mat& img) {
    int x_size = img.cols, y_size = img.rows;
    const uchar* in = img.ptr<uchar>();
    MT* r = &mag_(0);
    GT* dx = &dx_(0);
    GT* dy = &dy_(0);

    int i, j, pos;
    MT m, n;
    GT x, y, w = 0;
    uchar c, do_symmetry;
    const uchar* p;
    const uchar* cp;
    const uchar* bp = bp_ + 256;


    for (i = 3; i < y_size - 3; ++i)
      for (j = 3; j < x_size - 3; ++j) {
        pos = i * x_size + j;
        n = 100;
        p = in + (i - 3) * x_size + j - 1;
        cp = bp + in[pos];

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 3;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 5;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 6;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += 2;
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 6;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 5;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);
        p += x_size - 3;

        n += *(cp - *p++);
        n += *(cp - *p++);
        n += *(cp - *p);

        if (n <= max_no_) r[pos] = max_no_ - n;
      }

    for (i = 4; i < y_size - 4; ++i)
      for (j = 4; j < x_size - 4; ++j) {
        pos = i * x_size + j;
        if (r[pos] > 0) {
          m = r[pos];
          n = max_no_ - m;
          cp = bp + in[pos];

          if (n > 600) {
            p = in + (i - 3) * x_size + j - 1;
            x = 0;
            y = 0;

            c = *(cp - *p++);
            x -= c;
            y -= 3 * c;
            c = *(cp - *p++);
            y -= 3 * c;
            c = *(cp - *p);
            x += c;
            y -= 3 * c;
            p += x_size - 3;

            c = *(cp - *p++);
            x -= 2 * c;
            y -= 2 * c;
            c = *(cp - *p++);
            x -= c;
            y -= 2 * c;
            c = *(cp - *p++);
            y -= 2 * c;
            c = *(cp - *p++);
            x += c;
            y -= 2 * c;
            c = *(cp - *p);
            x += 2 * c;
            y -= 2 * c;
            p += x_size - 5;

            c = *(cp - *p++);
            x -= 3 * c;
            y -= c;
            c = *(cp - *p++);
            x -= 2 * c;
            y -= c;
            c = *(cp - *p++);
            x -= c;
            y -= c;
            c = *(cp - *p++);
            y -= c;
            c = *(cp - *p++);
            x += c;
            y -= c;
            c = *(cp - *p++);
            x += 2 * c;
            y -= c;
            c = *(cp - *p);
            x += 3 * c;
            y -= c;
            p += x_size - 6;

            c = *(cp - *p++);
            x -= 3 * c;
            c = *(cp - *p++);
            x -= 2 * c;
            c = *(cp - *p);
            x -= c;
            p += 2;
            c = *(cp - *p++);
            x += c;
            c = *(cp - *p++);
            x += 2 * c;
            c = *(cp - *p);
            x += 3 * c;
            p += x_size - 6;

            c = *(cp - *p++);
            x -= 3 * c;
            y += c;
            c = *(cp - *p++);
            x -= 2 * c;
            y += c;
            c = *(cp - *p++);
            x -= c;
            y += c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p++);
            x += c;
            y += c;
            c = *(cp - *p++);
            x += 2 * c;
            y += c;
            c = *(cp - *p);
            x += 3 * c;
            y += c;
            p += x_size - 5;

            c = *(cp - *p++);
            x -= 2 * c;
            y += 2 * c;
            c = *(cp - *p++);
            x -= c;
            y += 2 * c;
            c = *(cp - *p++);
            y += 2 * c;
            c = *(cp - *p++);
            x += c;
            y += 2 * c;
            c = *(cp - *p);
            x += 2 * c;
            y += 2 * c;
            p += x_size - 3;

            c = *(cp - *p++);
            x -= c;
            y += 3 * c;
            c = *(cp - *p++);
            y += 3 * c;
            c = *(cp - *p);
            x += c;
            y += 3 * c;

            if ((x * x + y * y) > (0.81 * n * n)) {
              do_symmetry = 0;
              float z = (x == 0) ? 1000000.0f : static_cast<float>(y) / static_cast<float>(x);
              if (z < 0) {
                z = -z;
                w = -1;
              } else
                w = 1;
              if (z < 0.25) { /* vert_edge */
                dy[pos] = 0;
                dx[pos] = 1;
              } else {
                if (z > 4.0) { /* hor_edge */
                  dy[pos] = 1;
                  dx[pos] = 0;
                } else { /* diag_edge */
                  if (w > 0) {
                    dy[pos] = 1;
                    dx[pos] = 1;
                  } else {
                    dy[pos] = -1;
                    dx[pos] = 1;
                  }
                }
              }

              // dy[pos] = sqrt(x); dx[pos] = sqrt(y);

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
            x += c;
            y += 9 * c;
            w += 3 * c;
            c = *(cp - *p++);
            y += 9 * c;
            c = *(cp - *p);
            x += c;
            y += 9 * c;
            w -= 3 * c;
            p += x_size - 3;

            c = *(cp - *p++);
            x += 4 * c;
            y += 4 * c;
            w += 4 * c;
            c = *(cp - *p++);
            x += c;
            y += 4 * c;
            w += 2 * c;
            c = *(cp - *p++);
            y += 4 * c;
            c = *(cp - *p++);
            x += c;
            y += 4 * c;
            w -= 2 * c;
            c = *(cp - *p);
            x += 4 * c;
            y += 4 * c;
            w -= 4 * c;
            p += x_size - 5;

            c = *(cp - *p++);
            x += 9 * c;
            y += c;
            w += 3 * c;
            c = *(cp - *p++);
            x += 4 * c;
            y += c;
            w += 2 * c;
            c = *(cp - *p++);
            x += c;
            y += c;
            w += c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p++);
            x += c;
            y += c;
            w -= c;
            c = *(cp - *p++);
            x += 4 * c;
            y += c;
            w -= 2 * c;
            c = *(cp - *p);
            x += 9 * c;
            y += c;
            w -= 3 * c;
            p += x_size - 6;

            c = *(cp - *p++);
            x += 9 * c;
            c = *(cp - *p++);
            x += 4 * c;
            c = *(cp - *p);
            x += c;
            p += 2;
            c = *(cp - *p++);
            x += c;
            c = *(cp - *p++);
            x += 4 * c;
            c = *(cp - *p);
            x += 9 * c;
            p += x_size - 6;

            c = *(cp - *p++);
            x += 9 * c;
            y += c;
            w -= 3 * c;
            c = *(cp - *p++);
            x += 4 * c;
            y += c;
            w -= 2 * c;
            c = *(cp - *p++);
            x += c;
            y += c;
            w -= c;
            c = *(cp - *p++);
            y += c;
            c = *(cp - *p++);
            x += c;
            y += c;
            w += c;
            c = *(cp - *p++);
            x += 4 * c;
            y += c;
            w += 2 * c;
            c = *(cp - *p);
            x += 9 * c;
            y += c;
            w += 3 * c;
            p += x_size - 5;

            c = *(cp - *p++);
            x += 4 * c;
            y += 4 * c;
            w -= 4 * c;
            c = *(cp - *p++);
            x += c;
            y += 4 * c;
            w -= 2 * c;
            c = *(cp - *p++);
            y += 4 * c;
            c = *(cp - *p++);
            x += c;
            y += 4 * c;
            w += 2 * c;
            c = *(cp - *p);
            x += 4 * c;
            y += 4 * c;
            w += 4 * c;
            p += x_size - 3;

            c = *(cp - *p++);
            x += c;
            y += 9 * c;
            w -= 3 * c;
            c = *(cp - *p++);
            y += 9 * c;
            c = *(cp - *p);
            x += c;
            y += 9 * c;
            w += 3 * c;

            float z = (y == 0) ? 1000000.0f : static_cast<float>(x) / static_cast<float>(y);
            if (z < 0.25) { /* vertical */
              dy[pos] = 0;
              dx[pos] = 1;
            } else {
              if (z > 4.0) { /* horizontal */
                dy[pos] = 1;
                dx[pos] = 0;
              } else { /* diagonal */
                if (w > 0) {
                  dy[pos] = -1;
                  dx[pos] = 1;
                } else {
                  dy[pos] = 1;
                  dx[pos] = 1;
                }
              }
            }
            // dy[pos] = (w > 0) ? -sqrt(x) : sqrt(x); dx[pos] = sqrt(y);
          }
        }
      }
  }
};
}  // namespace lsfm
