/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


/*
 *  Fast connected components line detection algorithm using patterns
 *  Version 1.1
 *  (C) by Benjamin Wassermann
 */

#pragma once

#include <lsd/lsd_cc_base.hpp>

namespace lsfm {
// Use complex decision algo to determine near pixels in cc search
static const int CP_FIND_NEAR_COMPLEX = 1;
// Enable corner rule
static const int CP_CORNER_RULE = 2;
// Use merge instead of split (faster but less precise)
static const int CP_MERGE = 4;

// constants
static const char CP_PATTERN_REVERSE = 1;
static const char CP_PATTERN_TERMINATE = 2;
static const char CP_PATTERN_IGNORE = 4;

template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = FitLine<EigenFit<FT, PT>>>
class LsdCP : public LsdCCBase<FT, LPT, PT, GRAD, FIT> {
  int flags_{}, min_pix_{}, max_gap_{}, pat_tol_{};
  FT err_dist_{};

  void init() {
    this->add("edge_min_pixels", std::bind(&LsdCP<FT, LPT, PT, GRAD, FIT>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels for line segment.");
    this->add("edge_max_gap", std::bind(&LsdCP<FT, LPT, PT, GRAD, FIT>::valueMaxGap, this, std::placeholders::_1),
              "Maximum pixel number of gaps.");
    this->add("split_error_distance",
              std::bind(&LsdCP<FT, LPT, PT, GRAD, FIT>::valueDistance, this, std::placeholders::_1),
              "Maximum error distance before line segment is split");
    this->add("edge_pattern_tolerance",
              std::bind(&LsdCP<FT, LPT, PT, GRAD, FIT>::valueTolerance, this, std::placeholders::_1),
              "Pattern tolerance, inication how strong a primitive can change before a new pattern is detected.");
    this->add("line_flags", std::bind(&LsdCP<FT, LPT, PT, GRAD, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - complex find near, 2 - corner rule, 4 - use merge mode instead "
              "of split mode.");
  }

  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::img_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::rows_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::cols_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::size_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::imageData_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::lsmap_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::seeds_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::points_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::emap_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::th_low_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::th_high_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::grad_;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::fit_;
  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;


 public:
  static const int CP_SOFT_CORNER;

  typedef FT value_type;
  typedef LPT<FT> line_point;
  typedef PT point_type;
  typedef typename GRAD::mag_type mag_type;
  typedef typename GRAD::grad_type grad_type;
  typedef typename LsdCCBase<FT, LPT, PT, GRAD, FIT>::Line Line;
  typedef typename LsdCCBase<FT, LPT, PT, GRAD, FIT>::LineVector LineVector;
  typedef typename LsdCCBase<FT, LPT, PT, GRAD, FIT>::LineSegment LineSegment;
  typedef typename LsdCCBase<FT, LPT, PT, GRAD, FIT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdCCBase<FT, LPT, PT, GRAD, FIT>::PointVector PointVector;


  struct PatternPrimitive {
    PatternPrimitive(ushort s = 0, char d = -1, char dn = -1) : size(s), dir(d), dir_next(dn) {}

    // primitive size
    ushort size{};
    // primitive direction
    char dir{};
    // direction change to next primitive
    char dir_next{};

    //! match PatternPrimitive by tolerance
    inline bool match(const PatternPrimitive& rhs, int tol) const {
      return dir == rhs.dir && (size == rhs.size || std::abs(size - rhs.size) <= tol);
    }

    //! compare operator
    inline bool operator==(const PatternPrimitive& rhs) const {
      return size == rhs.size && dir == rhs.dir && dir_next == rhs.dir_next;
    }

    inline bool operator!=(const PatternPrimitive& rhs) const { return !operator==(rhs); }

    //! convert direction d to x,y steps
    static const short* dir_step(char d) {
      static short steps[9][2] = {{0, 0}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
      return steps[static_cast<int>(d)];
    }

    //! convert direction difference to cyclic difference (e.g. 0 - 7 is not -7 but same as 0 - -1 = 1)
    static char dir_diff(char diff) {
      static char diffmap[] = {1, 2, 3, -4, -3, -2, -1, 0, 1, 2, 3, 4, -3, -2, -1};
      return diffmap[static_cast<int>(diff) + 7];
    }

    //! convert direction difference to cyclic difference
    static char dir_diff(char a, char b) { return dir_diff(static_cast<char>(a - b)); }

    //! convert direction difference to cyclic absolute difference
    static char dir_abs_diff(char diff) {
      static char diffmap[] = {1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1};
      return diffmap[static_cast<int>(diff) + 7];
    }

    //! convert direction difference to cyclic absolute difference
    static char dir_abs_diff(char a, char b) { return dir_abs_diff(static_cast<char>(a - b)); }
  };

  struct Pattern {
    Pattern(size_t b = 0, ushort s = 0, uchar f = 0) : beg(b), size(s), flags(f) {}

    Pattern(size_t b, ushort s, uchar f, ushort sp, char d = -1, char dn = -1)
        : beg(b), primitive(sp, d, dn), size(s), flags(f) {}

    Pattern(size_t b, ushort s, uchar f, const PatternPrimitive& pp) : beg(b), primitive(pp), size(s), flags(f) {}

    // start position in point list
    size_t beg{};
    // pattern primitive data
    PatternPrimitive primitive{};
    // number of points
    ushort size{};
    // pattern control flags
    uchar flags{};

    inline size_t begpos() const { return beg; }

    inline size_t endpos() const { return beg + size; }

    inline bool reverse() const { return flags & CP_PATTERN_REVERSE; }

    inline bool terminate() const { return (flags & CP_PATTERN_TERMINATE) != 0; }

    inline bool ignore() const { return (flags & CP_PATTERN_IGNORE) != 0; }
  };

  typedef std::vector<Pattern> PatternVector;

  // store all parameters of line segement
  class LineData {
    const PointVector* points_;
    const PatternVector* patterns_;

   public:
    LineData(size_t b = 0, unsigned int s = 0, const PointVector* po = 0, const PatternVector* pa = 0)
        : points_(po), patterns_(pa), pat_beg(b), pat_size(s) {}

    // start position in pattern list
    size_t pat_beg{};
    // number of patterns
    unsigned int pat_size{};

    inline size_t pattern_begpos() const { return pat_beg; }

    inline size_t pattern_endpos() const { return pat_beg + pat_size; }

    inline size_t pattern_size() const { return pat_size; }

    inline typename PatternVector::const_iterator pattern_begin() const {
      return patterns_->cbegin() + pattern_begpos();
    }

    inline typename PatternVector::const_iterator pattern_end() const { return patterns_->cbegin() + pattern_endpos(); }

    inline typename PatternVector::const_reverse_iterator pattern_rbegin() const {
      return PatternVector::const_reverse_iterator(pattern_end());
    }

    inline typename PatternVector::const_reverse_iterator pattern_rend() const {
      return PatternVector::const_reverse_iterator(pattern_begin());
    }

    inline const Pattern& pattern_front() const { return (*patterns_)[pattern_begpos()]; }

    inline const Pattern& pattern_back() const { return (*patterns_)[pattern_endpos() - 1]; }

    inline PatternVector patterns() const { return PatternVector(pattern_begin(), pattern_end()); }

    inline bool reverse() const { return pattern_front().reverse(); }

    inline size_t size() const { return pattern_back().endpos() - pattern_front().begpos(); }

    inline size_t begpos() const { return pattern_front().begpos(); }

    inline size_t endpos() const { return pattern_back().endpos(); }

    inline typename PointVector::const_iterator begin() const {
      return points_->cbegin() + static_cast<std::ptrdiff_t>(begpos());
    }

    inline typename PointVector::const_iterator end() const {
      return points_->cbegin() + static_cast<std::ptrdiff_t>(endpos());
    }

    inline typename PointVector::const_reverse_iterator rbegin() const {
      return PointVector::const_reverse_iterator(end());
    }

    inline typename PointVector::const_reverse_iterator rend() const {
      return PointVector::const_reverse_iterator(begin());
    }

    inline const PT& front() const { return (*points_)[begpos()]; }

    inline const PT& back() const { return (*points_)[endpos() - 1]; }


    inline PointVector points() const { return PointVector(begin(), end()); }

    inline PointVector ordered_points() const {
      return PointVector(reverse() ? rbegin() : begin(), reverse ? rend() : end());
    }

    inline FT magnitude(const cv::Mat& mag) const {
      FT m = 0;
      for_each(begin(), end(), [&](const PT& p) { m += sqrt(static_cast<FT>(mag.at<int>(p))); });
      return m / size();
    }
  };

  typedef std::vector<LineData> LineDataVector;

  //! Create a CPLineSegmentDetector object.
  //! @param th_high  Higher intensity threshold for Non Maxima Supression. Range (0..1] (0.004 ~ 1/255).
  //! @param th_low   Lower intensity threshold for Non Maxima Supression. Range (0..1] (0.004 ~ 1/255).
  //! @param min_pix      Minimum number of supporting pixels for line region. Range [2..X]
  //! @param max_gap      Maximum gap search distance. Range [0...X]
  //! @param err_dist     Error distance for pattern merge/split. Distance in Pixels. Range (0..X] (>0)
  //! @param pat_tol      Pattern tolerance: amount of allowed pixel number changes for primitives. Range [0..X]
  //! @param flags Flags for line detection
  //!                   CP_FIND_NEAR_COMPLEX - Use complex decision algo to determine near pixels in cc search
  //!                   CP_CORNER_RULE - Enable corner rule
  //!                   CP_MERGE - Use merge instead of split (faster but less precise)

  LsdCP(FT th_low = 0.004,
        FT th_high = 0.012,
        int min_pix = 10,
        int max_gap = 0,
        FT err_dist = 2,
        int pat_tol = 2,
        int flags = 0)
      : LsdCCBase<FT, LPT, PT, GRAD, FIT>(th_high, th_low),
        flags_(flags),
        min_pix_(min_pix),
        max_gap_(max_gap),
        pat_tol_(pat_tol),
        err_dist_(err_dist) {
    CV_Assert(pat_tol >= 0 && max_gap >= 0 && min_pix > 1 && th_high <= 1 && th_high > 0 && th_low <= 1 && th_low > 0 &&
              th_high >= th_low && err_dist > 0);
    init();
  }

  LsdCP(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdCP(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return min_pix_;
  }

  int minPixels() const { return min_pix_; }

  void minPixels(int mp) { min_pix_ = mp; }

  Value valueMaxGap(const Value& mg = Value::NAV()) {
    if (mg.type()) maxGap(mg.getInt());
    return max_gap_;
  }

  int maxGap() const { return max_gap_; }

  void maxGap(int mg) { max_gap_ = mg; }

  Value valueDistance(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.get<FT>());
    return err_dist_;
  }

  FT distance() const { return err_dist_; }

  void distance(FT d) { err_dist_ = d; }

  Value valueTolerance(const Value& pt = Value::NAV()) {
    if (pt.type()) tolerance(pt.getInt());
    return pat_tol_;
  }

  int tolerance() const { return pat_tol_; }

  void tolerance(int pt) { pat_tol_ = pt; }

  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  int flags() const { return flags_; }

  void flags(int f) { flags_ = f; }

  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::detect;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::lines;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::lineSegments;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::endPoints;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::imageDataDescriptor;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::imageData;

  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::points;
  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::indexes;

  const PatternVector& patternData() const { return patterns_; }

  EdgeSegmentVector patterns() const {
    EdgeSegmentVector ret;
    ret.resize(patterns_.size());
    for (size_t i = 0; i != patterns_.size(); ++i)
      ret[i] = EdgeSegment(patterns_[i].begpos(), patterns_[i].endpos(), patterns_[i].reverse() ? ES_REVERSE : ES_NONE);
    return ret;
  }

  const LineDataVector& lineDataSegments() const { return lineData_; }

  EdgeSegmentVector lineSupportSegments() const {
    EdgeSegmentVector ret;
    ret.resize(lineData_.size());
    for (size_t i = 0; i != lineData_.size(); ++i)
      ret[i] = EdgeSegment(lineData_[i].begpos(), lineData_[i].endpos(), lineData_[i].reverse ? ES_REVERSE : ES_NONE,
                           lineData_[i].id);
    return ret;
  }

  EdgeSegmentVector segments() const {
    EdgeSegmentVector ret;
    if (patterns_.empty()) return ret;
    ret.resize(patterns_.size() / 3);
    size_t start = patterns_.begin()->begpos();
    for_each(patterns_.begin(), patterns_.end(), [&](const Pattern& p) {
      if (p.terminate()) {
        ret.push_back(EdgeSegment(start, p.endpos(), p.reverse() ? ES_REVERSE : 0));
        start = p.endpos();
      }
    });
    return ret;
  }

  virtual void detect(const cv::Mat& image) final {
    img_ = image;
    CV_Assert(!img_.empty());

    // init vars
    rows_ = img_.rows;
    cols_ = img_.cols;
    size_ = cols_ * rows_;

    // clear data
    this->clearData();

    // call initial methods
    this->preprocess();
    // call cc method
    computePatterns();
    // merge/split patterns
    flags_& CP_MERGE ? mergePatterns() : splitPatterns();
    // create lines
    computeLines();
  }

  GRAD& grad() { return grad_; }

  const GRAD& grad() const { return grad_; }

 private:
  // list of all patterns in segments
  PatternVector patterns_{};
  // line data vector
  LineDataVector lineData_{};

  // LsdCP& operator= (const LsdCP&); // to quiet MSVC

  // private helper structs
  struct SearchState {
    SearchState(int i = 0, int xp = 0, int yp = 0, char d = -1) : idx(i), x(xp), y(yp), dir(d) {}

    int idx{}, x{}, y{};
    char dir{};
  };

  struct SegmentState {
    SegmentState(int i = 0, size_t p = 0, uchar r = 0, const SearchState& s = SearchState())
        : id(i), pos(p), rstate(r), sstate(s) {}

    int id{};
    size_t pos{};
    uchar rstate{};
    SearchState sstate{};
  };

  void computePatterns() {
    lsmap_.create(rows_, cols_, CV_32SC1);
    lsmap_.setTo(0);
    lsmap_.row(0).setTo(-1);
    lsmap_.col(0).setTo(-1);
    lsmap_.row(rows_ - 1).setTo(-1);
    lsmap_.col(cols_ - 1).setTo(-1);

    size_t size = static_cast<size_t>(static_cast<FT>(seeds_.size()) * th_high_ / th_low_);
    points_.clear();
    points_.reserve(size);
    patterns_.clear();
    patterns_.reserve(size / static_cast<size_t>(2 * pat_tol_));

    short dmapStore[28][4] = {{-1, -1, 0, 0},
                              {static_cast<short>(-1 - cols_), -1, -1, 1},
                              {static_cast<short>(-cols_), 0, -1, 2},
                              {static_cast<short>(1 - cols_), 1, -1, 3},
                              {1, 1, 0, 4},
                              {static_cast<short>(1 + cols_), 1, 1, 5},
                              {static_cast<short>(cols_), 0, 1, 6},
                              {static_cast<short>(-1 + cols_), -1, 1, 7},
                              {-1, -1, 0, 0},
                              {static_cast<short>(-1 - cols_), -1, -1, 1},
                              {static_cast<short>(-cols_), 0, -1, 2},
                              {static_cast<short>(1 - cols_), 1, -1, 3},
                              {1, 1, 0, 4},
                              {static_cast<short>(1 + cols_), 1, 1, 5},
                              {static_cast<short>(cols_), 0, 1, 6},
                              {static_cast<short>(-1 + cols_), -1, 1, 7},
                              {-1, -1, 0, 0},
                              {static_cast<short>(-1 - cols_), -1, -1, 1},
                              {static_cast<short>(-cols_), 0, -1, 2},
                              {static_cast<short>(1 - cols_), 1, -1, 3},
                              {1, 1, 0, 4},
                              {static_cast<short>(1 + cols_), 1, 1, 5},
                              {static_cast<short>(cols_), 0, 1, 6},
                              {static_cast<short>(-1 + cols_), -1, 1, 7},
                              {-1, -1, 0, 0},
                              {static_cast<short>(-1 - cols_), -1, -1, 1},
                              {static_cast<short>(-cols_), 0, -1, 2},
                              {static_cast<short>(1 - cols_), 1, -1, 3}};

    char diffmapStore[] = {1, 2, 3, -4, -3, -2, -1, 0, 1, 2, 3, 4, -3, -2, -1};
    char abs_diffmapStore[] = {1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1};

    char* pemap = emap_.template ptr<char>();
    int* plsmap = lsmap_.template ptr<int>();
    const mag_type* pmag = grad_.magnitude().template ptr<mag_type>();

    const short(*dmap)[4] = &dmapStore[8];
    const short(*pdmap)[4] = dmap;
    const char* diffmap = &diffmapStore[7];
    const char* abs_diffmap = &abs_diffmapStore[7];

    SegmentState seg(1);

    // check for pixel in ndir direction
    auto find_pixel = [&](SearchState& fs, char edir, char ndir) {
      const int dirIndex = static_cast<int>(ndir);
      int idx2 = fs.idx + pdmap[dirIndex][0];
      char edir2 = pemap[idx2];
      // pixel is already used
      if (edir2 < 0 || plsmap[idx2] || abs_diffmap[static_cast<int>(edir - edir2)] > 1) return false;
      fs.idx = idx2;
      fs.x += pdmap[dirIndex][1];
      fs.y += pdmap[dirIndex][2];
      fs.dir = static_cast<char>(dmap[dirIndex][3]);
      return true;
    };

    // find pixel near (fw + left, fw + right) current pixel
    auto find_near_simple = [&](SearchState& fs, char edir) {
      if (find_pixel(fs, edir, edir + 1)) return true;
      if (find_pixel(fs, edir, edir - 1)) return true;
      // if still no pixel was found, check if last direction was != edge map dir
      // and search again
      char diff = diffmap[static_cast<int>(fs.dir - edir)];
      if (std::abs(diff) == 1 && find_pixel(fs, edir, static_cast<char>(edir + (2 * diff)))) return true;

      return false;
    };

    // find pixel near (fw + left, fw + right) current pixel with some checking
    // for the better direction if there is a fork
    auto find_near_complex = [&](SearchState& fs, char edir) {
      SearchState tmp1 = fs;
      // also search one direction before
      if (find_pixel(tmp1, edir, edir + 1)) {
        // we also have to test if direction after is valid
        SearchState tmp2 = fs;
        if (find_pixel(tmp2, edir, edir - 1)) {
          // try to decide for one direction
          char edir1 = pemap[tmp1.idx], edir2 = pemap[tmp2.idx];
          // first try to check if one side has a better fit to the previous edir
          if (edir1 != edir2) {
            if (edir1 == edir) {
              fs = tmp1;
              return true;
            }
            if (edir2 == edir) {
              fs = tmp2;
              return true;
            }
          }
          // still no prove, try to get next pixels
          SearchState tmp3 = tmp1;
          // failed to get pixel, use other dir
          if (!find_pixel(tmp3, edir, edir) && !find_pixel(tmp3, edir, edir + 1)) {
            fs = tmp2;
            return true;
          }
          SearchState tmp4 = tmp2;
          if (!find_pixel(tmp4, edir, edir) && !find_pixel(tmp4, edir, edir - 1)) {
            fs = tmp1;
            return true;
          }
          // got pixels on both sides -> try to decide by difference to main dir or greater gradient magnitude
          if (abs_diffmap[static_cast<int>(edir - tmp3.dir)] > abs_diffmap[static_cast<int>(edir - tmp4.dir)] ||
              (abs_diffmap[static_cast<int>(edir - tmp3.dir)] == abs_diffmap[static_cast<int>(edir - tmp4.dir)] &&
               pmag[tmp2.idx] > pmag[tmp1.idx])) {
            fs = tmp2;
            return true;
          }
          // no decision found, or tmp3 has lesser error or tmp1 has greater gradient mag -> use edir + 1 (tmp1)
        }
        fs = tmp1;
        return true;
      }
      // and after
      if (find_pixel(fs, edir, edir - 1)) return true;

      // if still no pixel was found, check if last direction was != edge map dir
      // and search again
      char diff = diffmap[static_cast<int>(fs.dir - edir)];
      if (std::abs(diff) == 1 && find_pixel(fs, edir, static_cast<char>(edir + (2 * diff)))) return true;

      return false;
    };

    std::function<bool(SearchState & fs, char edir)> find_near = find_near_simple;
    if (flags_ & CP_FIND_NEAR_COMPLEX) find_near = find_near_complex;

    // find next pixel without gap detection and no thickness check
    auto find_next_no_check = [&](SearchState& fs) -> bool {
      char edir = pemap[fs.idx];

      // try to find next pixel direction of edge map
      if (find_pixel(fs, edir, edir)) return true;
      return find_near(fs, edir);
    };

    // check for thick lines and remove pixels
    auto check_dir = [&](const SearchState& ls, char ndir) -> void {
      int idx2 = ls.idx + pdmap[static_cast<int>(ndir)][0];
      if (pemap[idx2] < 0 || plsmap[idx2]) return;
      plsmap[idx2] = -2;
    };

    // find next pixel without gap detection
    auto find_next = [&](SearchState& fs) -> bool {
      char edir = pemap[fs.idx];

      SearchState ls = fs;
      // try to find next pixel direction of edge map
      if (find_pixel(fs, edir, edir)) {
        // check for bad pixels (only on odd dirs - diagonals)
        if (fs.dir % 2) {
          check_dir(ls, fs.dir - 1);
          check_dir(ls, fs.dir + 1);
        }
        return true;
      }

      return find_near(fs, edir);
    };

    auto move_idx = [&pemap, &pdmap](int& idx, char ndir) {
      idx += pdmap[static_cast<int>(ndir)][0];
      return pemap[idx];
    };

    auto find_gap = [&](char pdir, SearchState& s) -> bool {
      // get direction for gap detection
      char edir = pemap[s.idx];

      // set main direction
      if (edir != pdir && this->points_.size() > 1) {
        char edir2 = pemap[point2Index(*(this->points_.end() - 2), this->cols_)];
        // we have 3 directions!?
        if (edir2 != pdir && this->points_.size() > 2) {
          char edir3 = pemap[point2Index(*(this->points_.end() - 3), this->cols_)];
          if (edir2 == edir3 || pdir == edir3) {
            edir = edir3;
          }
          // still no match, keep edir
        } else
          edir = edir2;
      }


      int idx = s.idx;
      if (move_idx(idx, edir) == -2) return false;

      char ldir;
      int idxl = idx;
      int idxr = idx;

      int idxl2, idxr2;
      char rdir, tmp1, tmp2;

      for (int i = 0; i != max_gap_; ++i) {
        ldir = move_idx(idxl, edir);
        if (ldir == -2) return false;
        if (ldir > -1) {
          if (abs_diffmap[static_cast<int>(edir - ldir)] < 2) {
            idx = idxl;
            goto done;
          }
          return false;
        }
      }

      idxl = idx;
      if (move_idx(idxl, edir - 2) == -2) return false;
      if (move_idx(idxr, edir + 2) == -2) return false;

      idxl2 = idxl;
      tmp1 = move_idx(idxl2, edir - 2);
      idxr2 = idxr;
      tmp2 = move_idx(idxr2, edir + 2);

      for (int i = 0; i != max_gap_; ++i) {
        ldir = move_idx(idxl, edir);
        if (ldir > -1 && abs_diffmap[static_cast<int>(edir - ldir)] < 2) {
          idx = idxl;
          goto done;
        }

        rdir = move_idx(idxr, edir);
        if (rdir > -1 && abs_diffmap[static_cast<int>(edir - rdir)] < 2) {
          idx = idxr;
          goto done;
        }

        if (ldir != -1 || rdir != -1) return false;
      }

      if (tmp1 == -2 || tmp2 == -2) return false;

      for (int i = 0; i != max_gap_; ++i) {
        ldir = move_idx(idxl2, edir);
        if (ldir > -1 && abs_diffmap[static_cast<int>(edir - ldir)] < 2) {
          idx = idxl2;
          goto done;
        }

        rdir = move_idx(idxr2, edir);
        if (rdir > -1 && abs_diffmap[static_cast<int>(edir - rdir)] < 2) {
          idx = idxr2;
          goto done;
        }

        if (ldir != -1 || rdir != -1) return false;
      }
      return false;

    done:
      if (plsmap[idx]) return false;
      int lmag = pmag[s.idx];
      s.dir = edir;
      s.idx = idx;
      s.x = static_cast<int>(idx % this->cols_);
      s.y = static_cast<int>(idx / this->cols_);
      // sanity check
      // if (std::abs(lmag - pmag[s.idx]) > 99)
      //     return false;
      SearchState tmp = s;
      return (!find_next_no_check(tmp) ||
              (abs_diffmap[static_cast<int>(edir - pemap[tmp.idx])] < 2 && std::abs(lmag - pmag[tmp.idx]) < 500));
    };

    // search connected components in one direction
    auto search_no_cr = [&](Pattern lp, Pattern p, char ls_dir) {
      SearchState fs, fs2;
      bool next = true;

      auto checkEnd = [&](uchar flags) {
        // only need to add current pattern with termination flag
        if (lp.primitive.size == 0) {
          p.size = static_cast<ushort>(this->points_.size() - p.beg);
          p.flags = seg.rstate | flags;
          patterns_.push_back(p);
        }
        // current pattern fits to last, merge and reset
        else if (lp.primitive.match(p.primitive, pat_tol_)) {
          lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          lp.flags = seg.rstate | flags;
          patterns_.push_back(lp);
          lp = Pattern();
        }
        // add last and current pattern and reset
        else {
          lp.flags = seg.rstate;
          patterns_.push_back(lp);
          lp = Pattern();

          p.size = static_cast<ushort>(this->points_.size() - p.beg);
          p.flags = seg.rstate | flags;
          patterns_.push_back(p);
        }
      };

      // find next pixels
      while (next) {
        next = find_next(fs = seg.sstate);
        // if we have more than two pixels and a direction change, check pattern
        if (p.primitive.size > 1 && ls_dir != seg.sstate.dir) {
          // get new direction
          p.primitive.dir_next = seg.sstate.dir;

          // if direction change is to big, stop pattern
          if (abs_diffmap[static_cast<int>(ls_dir - seg.sstate.dir)] > 1) {
            checkEnd(0);
          }
          // if no last pattern exist, just store current pattern as last and continue
          else if (lp.primitive.size == 0) {
            lp = p;
            lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          } else if (lp.primitive.match(p.primitive, pat_tol_)) {
            // current pattern equals last pattern, update lp size and primitive size
            if (lp.primitive.dir_next == p.primitive.dir_next) {
              float repeat = static_cast<float>(lp.size) / lp.primitive.size + 1;
              lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
              lp.primitive.size = static_cast<ushort>(round(lp.size / repeat));
            }
            // no more repeat, but current element fits to last, add last pattern and reset to no last pattern
            else {
              lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
              lp.flags = seg.rstate;
              patterns_.push_back(lp);
              lp = Pattern();
            }
          }
          // no repeat, no fit -> store last pattern to list and replace with current pattern
          else {
            //  check if last pattern fits to current and previous  -> split
            if (lp.primitive.size == lp.size && lp.size > 1 && patterns_.size() &&
                lp.primitive.dir == p.primitive.dir && lp.primitive.dir_next == p.primitive.dir_next) {
              Pattern& blp = patterns_.back();
              if (!blp.terminate() && lp.primitive.dir == blp.primitive.dir) {
                unsigned int s = blp.primitive.size + p.primitive.size;
                if (lp.size <= s + static_cast<unsigned int>(2 * pat_tol_) &&
                    lp.size >= s - static_cast<unsigned int>(2 * pat_tol_)) {
                  unsigned int blpadd =
                      static_cast<unsigned int>(round(static_cast<double>(lp.size * blp.primitive.size) / s));
                  blp.size += static_cast<ushort>(blpadd);
                  p.beg = blp.endpos();
                  goto update_lp;
                }
              }
            }

            lp.flags = seg.rstate;
            patterns_.push_back(lp);
          update_lp:
            lp = p;
            lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          }
          // reset current pattern
          p = Pattern(this->points_.size());
        }
        // update main direction of pattern
        else {
          p.primitive.dir = seg.sstate.dir;
        }

        // set map and add point to list
        plsmap[seg.sstate.idx] = seg.id;
        this->points_.push_back(PT(seg.sstate.x, seg.sstate.y));

        // update pattern pixel count and segments states
        ++p.primitive.size;

        if (max_gap_ > 0 && !next && p.primitive.size > 2) {
          next = find_gap(p.primitive.dir, fs = seg.sstate);
          if (next) {
            checkEnd(0);
            p = Pattern(this->points_.size());
          }
        }

        ls_dir = seg.sstate.dir;
        seg.sstate = fs;
      }

      checkEnd(CP_PATTERN_TERMINATE);
    };

    // search connected components in one direction
    auto search_cr = [&](Pattern lp, Pattern p, char ls_dir) {
      SearchState fs, fs2;
      bool next = true;

      auto checkEnd = [&](uchar flags) {
        // only need to add current pattern with termination flag
        if (lp.primitive.size == 0) {
          p.size = static_cast<ushort>(this->points_.size() - p.beg);
          p.flags = seg.rstate | flags;
          patterns_.push_back(p);
        }
        // current pattern fits to last, merge and reset
        else if (lp.primitive.match(p.primitive, pat_tol_)) {
          lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          lp.flags = seg.rstate | flags;
          patterns_.push_back(lp);
          lp = Pattern();
        }
        // add last and current pattern and reset
        else {
          lp.flags = seg.rstate;
          patterns_.push_back(lp);
          lp = Pattern();

          p.size = static_cast<ushort>(this->points_.size() - p.beg);
          p.flags = seg.rstate | flags;
          patterns_.push_back(p);
        }
      };

      // find next pixels
      while (next) {
        next = find_next(fs = seg.sstate);
        // if we have more than two pixels and a direction change, check pattern
        if (p.primitive.size > 1 && ls_dir != seg.sstate.dir) {
          // get new direction
          p.primitive.dir_next = seg.sstate.dir;
          int diff_s = abs_diffmap[static_cast<int>(ls_dir - seg.sstate.dir)];

          // check corner rule
          if (next) {
            int diff_fs = abs_diffmap[static_cast<int>(ls_dir - fs.dir)];
            //                           |        |
            //                           |       /     |
            // check for hard corners  _/   or _|  or _|
            if (find_next(fs2 = fs)) {
              int diff_fs2 = abs_diffmap[static_cast<int>(ls_dir - fs2.dir)];
              if (diff_fs > 1) {
                if (diff_s == 2) {
                  checkEnd(CP_PATTERN_TERMINATE);
                  return;
                }

                if (diff_fs2 > 1) {
                  //      \     _
                  //       \     \    __
                  // case _/    _/    _/
                  if (diff_fs > 2 && diff_fs2 > 2) {
                    // add s to current pattern
                    // set map and add point to list
                    plsmap[seg.sstate.idx] = seg.id;
                    this->points_.push_back(PT(seg.sstate.x, seg.sstate.y));
                    // update pattern pixel count and segments states
                    ++p.primitive.size;
                  }
                  checkEnd(CP_PATTERN_TERMINATE);
                  return;
                }
              }
              if (diff_s == 2 && seg.sstate.dir == fs2.dir) {
                checkEnd(CP_PATTERN_TERMINATE);
                return;
              }
            }
            // break even if single point is left, because hard corner _|
            else if (diff_fs > 1) {
              checkEnd(CP_PATTERN_TERMINATE);
              return;
            }
          }

          // if direction change is to big, stop pattern
          if (diff_s > 1) {
            checkEnd(0);
          }
          // if no last pattern exist, just store current pattern as last and continue
          else if (lp.primitive.size == 0) {
            lp = p;
            lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          } else if (lp.primitive.match(p.primitive, pat_tol_)) {
            // current pattern equals last pattern, update lp size and primitive size
            if (lp.primitive.dir_next == p.primitive.dir_next) {
              float repeat = static_cast<float>(lp.size) / lp.primitive.size + 1;
              lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
              lp.primitive.size = static_cast<ushort>(round(lp.size / repeat));
            }
            // no more repeat, but current element fits to last, add last pattern and reset to no last pattern
            else {
              lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
              lp.flags = seg.rstate;
              patterns_.push_back(lp);
              lp = Pattern();
            }
          }
          // no repeat, no fit -> store last pattern to list and replace with current pattern
          else {
            //  check if last pattern fits to current and previous  -> split
            if (lp.primitive.size == lp.size && lp.size > 1 && patterns_.size() &&
                lp.primitive.dir == p.primitive.dir && lp.primitive.dir_next == p.primitive.dir_next) {
              Pattern& blp = patterns_.back();
              if (!blp.terminate() && lp.primitive.dir == blp.primitive.dir) {
                unsigned int s = blp.primitive.size + p.primitive.size;
                if (lp.size <= s + static_cast<unsigned int>(2 * pat_tol_) &&
                    lp.size >= s - static_cast<unsigned int>(2 * pat_tol_)) {
                  unsigned int blpadd =
                      static_cast<unsigned int>(round(static_cast<double>(lp.size * blp.primitive.size) / s));
                  blp.size += static_cast<ushort>(blpadd);
                  p.beg = blp.endpos();
                  goto update_lp;
                }
              }
            }

            lp.flags = seg.rstate;
            patterns_.push_back(lp);
          update_lp:
            lp = p;
            lp.size = static_cast<ushort>(this->points_.size() - lp.beg);
          }
          // reset current pattern
          p = Pattern(this->points_.size());
        }
        // update main direction of pattern
        else {
          p.primitive.dir = seg.sstate.dir;
        }

        // set map and add point to list
        plsmap[seg.sstate.idx] = seg.id;
        this->points_.push_back(PT(seg.sstate.x, seg.sstate.y));

        // update pattern pixel count and segments states
        ++p.primitive.size;

        if (max_gap_ > 0 && !next && p.primitive.size > 2) {
          next = find_gap(p.primitive.dir, fs = seg.sstate);
          if (next) {
            checkEnd(0);
            p = Pattern(this->points_.size());
          }
        }

        ls_dir = seg.sstate.dir;
        seg.sstate = fs;
      }

      checkEnd(CP_PATTERN_TERMINATE);
    };

    std::function<void(Pattern lp, Pattern p, char ls_dir)> search = search_no_cr;
    if (flags_ & CP_CORNER_RULE) search = search_cr;

    auto pattern_reverse = [&](typename PatternVector::iterator beg, typename PatternVector::iterator end,
                               size_t po_size, uchar flag) {
      size_t po_beg = beg->beg;
      size_t po_end = po_beg + po_size;
      std::reverse(this->points_.begin() + static_cast<std::ptrdiff_t>(po_beg),
                   this->points_.begin() + static_cast<std::ptrdiff_t>(po_end));
      while (beg < end) {
        --end;
        Pattern tmp = *beg;
        end->beg = po_beg;
        end->flags = flag;
        po_beg += end->size;
        *beg = *end;
        po_end -= tmp.size;
        tmp.beg = po_end;
        tmp.flags = flag;
        *end = tmp;
        ++beg;
      }
    };

    for_each(seeds_.begin(), seeds_.end(), [&](const PT& pidx) {
      int idx = static_cast<int>(point2Index(pidx, cols_));
      if (plsmap[idx]) return;

      seg.sstate = SearchState(idx, getX(pidx), getY(pidx), pemap[idx]);
      seg.pos = this->points_.size();

      Pattern lp, p(this->points_.size());
      SearchState ls;

      const short(*fwdmap)[4] = dmap;
      const short(*rvdmap)[4] = dmap + 4;

      // CV_Assert(seg.sstate.x != 83 || seg.sstate.y != 61);
      switch (seg.sstate.dir) {
        case 0:  // <---x---> : fw: <---, rv: --->
        case 7:
          // just switch fw with rv
          fwdmap = rvdmap;
          rvdmap = dmap;
          [[fallthrough]];
        case 3:  // <---x---> : rv: <---, fw: --->
        case 4: {
          SearchState fs = seg.sstate, s = seg.sstate;
          pdmap = fwdmap;
          // check for fw patterns
          if (!find_next_no_check(fs)) {
            // no fw patterns found, jump to rv search
            goto rv;
          }

          pdmap = rvdmap;
          // check for rv patterns
          if (!find_next_no_check(s)) {
            // if no rv patterns found, jump to normal fw search
            goto fw;
          }

          // check for rv patterns
          ls = s;
          if (!find_next_no_check(ls)) {
            seg.sstate = s;
            // if no rv patterns found, jump to normal fw search
            goto fw;
          }

          // do reverse search from s
          size_t ppos1 = patterns_.size();
          fs = seg.sstate;
          seg.sstate = s;
          seg.sstate.dir = ls.dir;
          seg.rstate = fwdmap != dmap;
          search(lp, p, s.dir);

          // closed check
          if (this->points_.back() == PT(fs.x, fs.y)) {
            seg.rstate = dmap != fwdmap;
          } else {
            pattern_reverse(patterns_.begin() + static_cast<std::ptrdiff_t>(ppos1),
                            patterns_.begin() + static_cast<std::ptrdiff_t>(patterns_.size()),
                            this->points_.size() - seg.pos, static_cast<uchar>(fwdmap == dmap));

            // get current pattern
            p = patterns_.back();
            if (p.size != p.primitive.size) {
              lp = p;
              lp.size -= p.primitive.size;
              p.size = 0;
              p.beg = lp.beg + lp.size;
            }
            p.size = 0;
            patterns_.pop_back();
            pdmap = fwdmap;
            // correct seg.sstate by pos of original sstate and dir of s
            fs.dir = s.dir;
            seg.sstate = fs;
            seg.rstate = dmap == fwdmap;
            // continue search
            search(lp, p, ls.dir);
          }
        } break;
        case 1:
        case 2:  // rv
        rv:
          seg.rstate = dmap != fwdmap;
          pdmap = rvdmap;
          search(lp, p, ls.dir);
          break;
        case 5:
        case 6:  // fw
        fw:
          seg.rstate = dmap == fwdmap;
          pdmap = fwdmap;
          search(lp, p, ls.dir);
          break;
      }

      if (this->points_.size() - seg.pos > 1)
        ++seg.id;
      else {
        this->points_.pop_back();
        patterns_.pop_back();
        plsmap[seg.sstate.idx] = -2;
      }
    });
  }

  void mergePatterns() {
    lineData_.clear();
    lineData_.reserve(patterns_.size() / 2);

    bool lterm = true;
    PT start;
    LineData ld(0, 0, &points_, &patterns_);
    size_t idx = 0;
    for_each(patterns_.begin(), patterns_.end(), [&](const Pattern& p) {
      if (lterm) {
        ld.pat_beg = idx;
        start = this->points_[p.begpos()];
        lterm = false;
      } else {
        PT mid = this->points_[p.begpos()], mid2 = this->points_[p.begpos() - 1], end = this->points_[p.endpos() - 1];

        // get direction of line
        PT n = end - start;
        // get normal
        int tmp = getX(n);
        getX(n) = -getY(n);
        getY(n) = tmp;
        FT err = err_dist_ * l2_norm<FT>(n);

        PT a = mid - start, a2 = mid2 - start;
        if (std::abs(static_cast<FT>(getX(n) * getX(a) + getY(n) * getY(a))) > err ||
            std::abs(static_cast<FT>(getX(n) * getX(a2) + getY(n) * getY(a2))) > err) {
          ld.pat_size = static_cast<unsigned int>(idx - ld.pat_beg);
          if (ld.size() >= static_cast<size_t>(min_pix_)) {
            lineData_.push_back(ld);
          }
          ld.pat_beg = idx;
          start = this->points_[p.begpos()];
        }
      }
      ++idx;

      if (p.terminate()) {
        ld.pat_size = static_cast<unsigned int>(idx - ld.pat_beg);
        if (ld.size() >= static_cast<size_t>(min_pix_)) {
          lineData_.push_back(ld);
        }
        lterm = true;
      }
    });
  }

  void splitPatterns() {
    char abs_diffmapStore[] = {1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1};
    const char* abs_diffmap = &abs_diffmapStore[7];

    lineData_.clear();
    lineData_.reserve(patterns_.size() / 2);
    typedef typename PatternVector::const_iterator PatternIter;
    PatternIter pbeg = patterns_.begin();

    // recursive solution
    std::function<void(PatternIter, PatternIter)> search;
    search = [&](PatternIter beg, PatternIter end) {
      if (beg >= end) return;

      if (end - beg == 1) {
        if (static_cast<int>(beg->size) >= this->min_pix_)
          lineData_.push_back(LineData(static_cast<size_t>(beg - pbeg), 1, &this->points_, &this->patterns_));
        return;
      }

      --end;
      PatternIter p_idx, fbeg = beg;
      bool no_gap = false;

      const PT& first = this->points_[beg->begpos()];
      // get direction of line
      PT n = (this->points_[end->endpos() - 1] - first), a, a2;
      // get normal
      int max_h = getX(n), max_count = 0, h1, h2;
      getX(n) = -getY(n);
      getY(n) = max_h;
      max_h = 0;

      a = this->points_[beg->endpos() - 1] - first;
      if ((h2 = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
        max_h = h2;
        p_idx = beg + 1;
        no_gap = true;
      }

      ++beg;
      for (; beg < end; ++beg) {
        a = this->points_[beg->begpos()] - first;
        if ((h1 = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
          max_h = h1;
          p_idx = beg;
          no_gap = true;
          max_count = 0;
        } else if (h1 != max_h)
          no_gap = false;


        a = this->points_[beg->endpos() - 1] - first;
        if ((h2 = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
          max_h = h2;
          p_idx = beg + 1;
          no_gap = true;
          max_count = 0;
        } else if (no_gap && h2 == max_h && h1 == max_h)
          ++max_count;
        else
          no_gap = false;
      }

      a = this->points_[beg->begpos()] - first;
      if ((h1 = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
        max_h = h1;
        p_idx = beg;
        no_gap = true;
        max_count = 0;
      }

      ++end;

      if (static_cast<FT>(max_h) > err_dist_ * l2_norm<FT>(n)) {
        PatternIter max_end = p_idx;

        // corner check
        if (max_count != 0) {
          PatternIter e2 = p_idx + max_count;
          if (abs_diffmap[static_cast<int>((e2 - 1)->primitive.dir - e2->primitive.dir)] >
              abs_diffmap[static_cast<int>((p_idx - 1)->primitive.dir - p_idx->primitive.dir)])
            max_end = e2;
        }
        search(fbeg, max_end);
        search(max_end, end);
      } else
        lineData_.push_back(LineData(static_cast<size_t>(fbeg - pbeg), static_cast<unsigned int>(beg - fbeg) + 1,
                                     &this->points_, &this->patterns_));
    };

    PatternIter idx = patterns_.begin(), start = idx, end = patterns_.end();
    for (; idx != end; ++idx) {
      if (idx->terminate()) {
        search(start, idx + 1);
        start = idx + 1;
      }
    }
  }

  void computeLines() {
    lineSegments_.reserve(static_cast<size_t>(lineData_.size()));
    typedef typename PointVector::const_iterator const_iter;

    for_each(lineData_.begin(), lineData_.end(), [&](LineData& ldata) {
      if (ldata.size() < static_cast<size_t>(min_pix_)) return;
      const_iter beg = ldata.begin(), end = ldata.end();

      const PT& first = ldata.reverse() ? *beg : *(end - 1);
      const PT& last = ldata.reverse() ? *(end - 1) : *beg;

      Line l;
      fit_.apply(beg, end, l);

      // correct direction of line
      /*int epnx = getY(first) - getY(last);
      int epny = getX(last) - getX(first);
      if (epnx * l.normalX() + epny * l.normalY() < 0)
          l.normalFlip();*/

      lineSegments_.push_back(LineSegment(l, first, last));
    });
  }
};

}  // namespace lsfm
