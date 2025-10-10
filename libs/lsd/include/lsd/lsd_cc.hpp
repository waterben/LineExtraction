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
 *  Fast connected components line detection algorithm
 *  Version 1.3
 *  (C) by Benjamin Wassermann
 */

#pragma once

#include <edge/edge_segment.hpp>
#include <lsd/lsd_cc_base.hpp>


namespace lsfm {
// Use complex decision algo to determine near pixels in cc search
static const int CC_FIND_NEAR_COMPLEX = 1;
// Enable corner rule
static const int CC_CORNER_RULE = 2;
// Add thickness of lines (only on thick pixels)
static const int CC_ADD_THICK_PIXELS = 4;

template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = FitLine<EigenFit<FT, PT>>>
class LsdCC : public LsdCCBase<FT, LPT, PT, GRAD, FIT> {
  int flags_, min_pix_, max_gap_;
  FT err_dist_;

  void init() {
    this->add("edge_min_pixels", std::bind(&LsdCC<FT, LPT, PT, GRAD, FIT>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels for line segment.");
    this->add("edge_max_gap", std::bind(&LsdCC<FT, LPT, PT, GRAD, FIT>::valueMaxGap, this, std::placeholders::_1),
              "Maximum pixel number of gaps.");
    this->add("split_error_distance",
              std::bind(&LsdCC<FT, LPT, PT, GRAD, FIT>::valueDistance, this, std::placeholders::_1),
              "Maximum error distance before line segment is split.");
    this->add("line_flags", std::bind(&LsdCC<FT, LPT, PT, GRAD, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - complex find near, 2 - corner rule, 4 - add thick pixels");
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

  using LsdCCBase<FT, LPT, PT, GRAD, FIT>::clearData;

 public:
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


  // store all parameters of line segement
  class LineData {
    const PointVector* points_;

   public:
    LineData(int i = 0, size_t b = 0, size_t e = 0, bool r = false, const PointVector* v = 0)
        : points_(v), id(i), p_beg(b), p_end(e), reverse(r) {}

    // segment id of lsmap
    int id{};

    // start / end position in point list (supporting points)
    size_t p_beg{}, p_end{};

    // order of supporting points
    bool reverse{};

    // number of supporting points
    inline size_t size() const { return p_end - p_beg; }

    inline size_t begpos() const { return p_beg; }

    inline size_t endpos() const { return p_end; }

    inline typename PointVector::const_iterator begin() const {
      return points_->cbegin() + static_cast<std::ptrdiff_t>(p_beg);
    }

    inline typename PointVector::const_iterator end() const {
      return points_->cbegin() + static_cast<std::ptrdiff_t>(p_end);
    }

    inline typename PointVector::const_reverse_iterator rbegin() const {
      return PointVector::const_reverse_iterator(end());
    }

    inline typename PointVector::const_reverse_iterator rend() const {
      return PointVector::const_reverse_iterator(begin());
    }

    inline const PT& front() const { return (*points_)[begpos()]; }

    inline const PT& back() const { return (*points_)[endpos() - 1]; }


    inline PointVector points() const { PointVector(begin(), end()); }

    inline PointVector ordered_points() const {
      return reverse ? PointVector(rbegin(), rend()) : PointVector(begin(), end());
    }
  };

  typedef std::vector<LineData> LineDataVector;


  //!
  //! Create a LsdCC object.
  //! @param th_high  Higher intensity threshold for Non Maxima Supression. Range (0..1] (0.004 ~ 1/255).
  //! @param th_low   Lower intensity threshold for Non Maxima Supression. Range (0..1] (0.004 ~ 1/255).
  //! @param min_pix      Minimum number of supporting pixels for line segment. Range [2..X]
  //! @param max_gap      Maximum gap search distance. Range [0..X]
  //! @param err_dist     Error distance for segment splitting. Distance in Pixels. Range (0..X] (>0)
  //! @param flags Flags for line detection
  //!                   CC_FIND_NEAR_COMPLEX - Use complex decision algo to determine near pixels in cc search
  //!                   CC_CORNER_RULE - Enable corner rule
  //!                   CC_ADD_THICK_PIXELS - Add pixel of lines makeing them "thick" -> can result in higher
  //!                                         precision but also in more "false" splits.
  LsdCC(FT th_low = static_cast<FT>(0.004),
        FT th_high = static_cast<FT>(0.012),
        int min_pix = 10,
        int max_gap = 0,
        FT err_dist = 2,
        int flags = 0)
      : LsdCCBase<FT, LPT, PT, GRAD, FIT>(th_low, th_high),
        flags_(flags),
        min_pix_(min_pix),
        max_gap_(max_gap),
        err_dist_(err_dist),
        lineData_(),
        segments_() {
    CV_Assert(max_gap >= 0 && min_pix > 1 && th_high <= 1 && th_high > 0 && th_low <= 1 && th_low > 0 &&
              th_high >= th_low && err_dist > 0);

    init();
  }

  LsdCC(ValueManager::InitializerList options)
      : flags_(0), min_pix_(10), max_gap_(0), err_dist_(2), lineData_(), segments_() {
    init();
    this->value(options);
  }

  LsdCC(const ValueManager::NameValueVector& options)
      : flags_(0), min_pix_(10), max_gap_(0), err_dist_(2), lineData_(), segments_() {
    init();
    this->value(options);
  }

  LsdCC(const LsdCC& rhs)
      : LsdCCBase<FT, LPT, PT, GRAD, FIT>(rhs.th_low_, rhs.th_high_),
        flags_(rhs.flags_),
        min_pix_(rhs.min_pix_),
        max_gap_(rhs.max_gap_),
        err_dist_(rhs.err_dist_),
        lineData_(),
        segments_() {
    CV_Assert(max_gap_ >= 0 && min_pix_ > 1 && th_high_ <= 1 && th_high_ > 0 && th_low_ <= 1 && th_low_ > 0 &&
              th_high_ >= th_low_ && err_dist_ > 0);

    init();
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

  virtual void detect(const cv::Mat& image) final {
    img_ = image;
    CV_Assert(!img_.empty());

    // init vars
    rows_ = img_.rows;
    cols_ = img_.cols;
    size_ = cols_ * rows_;

    // clear data
    clearData();

    // call initial methods
    this->preprocess();
    // call segmentation methods
    computeSeg();
    splitSeg();
    // create lines
    computeLines();
  }


  const LineDataVector& lineDataSegments() const { return segments_; }

  EdgeSegmentVector lineSupportSegments() const {
    EdgeSegmentVector ret;
    ret.resize(segments_.size());
    for (size_t i = 0; i != segments_.size(); ++i)
      ret[i] = EdgeSegment(segments_[i].begpos(), segments_[i].endpos(), segments_[i].reverse ? ES_REVERSE : ES_NONE,
                           segments_[i].id);
    return ret;
  }

  EdgeSegmentVector segments() const {
    EdgeSegmentVector ret;
    ret.resize(lineData_.size());
    for (size_t i = 0; i != lineData_.size(); ++i)
      ret[i] = EdgeSegment(lineData_[i].begpos(), lineData_[i].endpos(), lineData_[i].reverse ? ES_REVERSE : ES_NONE);
    return ret;
  }

  GRAD& grad() { return grad_; }

  const GRAD& grad() const { return grad_; }

 private:
  // line segment vector
  LineDataVector lineData_;
  LineDataVector segments_;

  // detect connected edges
  void computeSeg() {
    lsmap_.create(rows_, cols_, CV_32SC1);
    lsmap_.setTo(0);
    lsmap_.row(0).setTo(-1);
    lsmap_.col(0).setTo(-1);
    lsmap_.row(rows_ - 1).setTo(-1);
    lsmap_.col(cols_ - 1).setTo(-1);

    size_t size = static_cast<size_t>(static_cast<FT>(seeds_.size()) * th_high_ / th_low_);
    points_.clear();
    points_.reserve(size);
    segments_.clear();
    segments_.reserve(static_cast<size_t>(size / static_cast<size_t>(min_pix_ * min_pix_) + 100));

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

    LineData seg(1, 0, 0, false, &this->points_);

    // simple internal structure
    struct State {
      State(int i = 0, int xp = 0, int yp = 0, char d = 0) : idx(i), x(xp), y(yp), dir(d) {}

      int idx{}, x{}, y{};
      char dir{};
    };

    State s{};

    // find next pixel by given direction
    auto find_pixel = [&](State& fs, char edir, char ndir) {
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

    // check for thick lines and add pixels
    auto check_dir_add = [&](const State& ls, char ndir) -> void {
      const int dirIndex = static_cast<int>(ndir);
      int idx2 = ls.idx + pdmap[dirIndex][0];
      if (pemap[idx2] < 0 || plsmap[idx2]) return;
      plsmap[idx2] = seg.id;
      this->points_.push_back(PT(ls.x + pdmap[dirIndex][1], ls.y + pdmap[dirIndex][2]));
    };

    // check for thick lines and remove pixels
    auto check_dir_remove = [&](const State& ls, char ndir) -> void {
      int idx2 = ls.idx + pdmap[static_cast<int>(ndir)][0];
      if (pemap[idx2] < 0 || plsmap[idx2]) return;
      plsmap[idx2] = -2;
    };

    std::function<void(const State& ls, char ndir)> check_dir = check_dir_remove;
    if (flags_ & CC_ADD_THICK_PIXELS) check_dir = check_dir_add;

    // find pixel near (fw + left, fw + right) current pixel
    auto find_near_simple = [&](State& fs, char edir) {
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
    auto find_near_complex = [&](State& fs, char edir) {
      State tmp1 = fs;
      // also search one direction before
      if (find_pixel(tmp1, edir, edir + 1)) {
        // we also have to test if direction after is valid
        State tmp2 = fs;
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
          State tmp3 = tmp1;
          // failed to get pixel, use other dir
          if (!find_pixel(tmp3, edir, edir) && !find_pixel(tmp3, edir, edir + 1)) {
            fs = tmp2;
            return true;
          }
          State tmp4 = tmp2;
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

    std::function<bool(State & fs, char edir)> find_near = find_near_simple;
    if (flags_ & CC_FIND_NEAR_COMPLEX) find_near = find_near_complex;

    // simple algo to jump over small gaps
    // auto find_gap = [&](State &fs, char edir) {
    auto find_gap = [&](State& fs, char edir) -> bool {
      auto add_gap_pixel = [&](char ndir) {
        const int dirIndex = static_cast<int>(ndir);
        int idx2 = fs.idx + pdmap[dirIndex][0];
        char edir2 = pemap[idx2];
        // pixel is already used
        if (edir2 < 0 || plsmap[idx2] || edir != edir2) return false;
        fs.idx = idx2;
        fs.x += pdmap[dirIndex][1];
        fs.y += pdmap[dirIndex][2];
        fs.dir = static_cast<char>(dmap[dirIndex][3]);
        return true;
      };

      for (int i = 0; i != max_gap_; ++i) {
        const int dirIndex = static_cast<int>(edir);
        fs.idx += pdmap[dirIndex][0];
        fs.x += pdmap[dirIndex][1];
        fs.y += pdmap[dirIndex][2];

        if (plsmap[fs.idx]) break;
        if (add_gap_pixel(edir)) return true;
        if (add_gap_pixel(edir + 1)) return true;
        if (add_gap_pixel(edir - 1)) return true;
      }

      return false;
    };

    // find next pixel without gap detection and no thickness check
    auto find_next_no_check = [&](State& fs) -> bool {
      char edir = pemap[fs.idx];

      // try to find next pixel direction of edge map
      if (find_pixel(fs, edir, edir)) return true;
      return find_near(fs, edir);
    };

    // find next pixel without gap detection
    auto find_next_no_gap = [&](State& fs) -> bool {
      char edir = pemap[fs.idx];

      State ls = fs;
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

    // find next pixel with gap detection
    auto find_next_gap = [&](State& fs) -> bool {
      char edir = pemap[fs.idx];

      State ls = fs;
      // try to find next pixel direction of edge map
      if (find_pixel(fs, edir, edir)) {
        // check for bad pixels (only on odd dirs - diagonals)
        if (fs.dir % 2) {
          check_dir(ls, fs.dir - 1);
          check_dir(ls, fs.dir + 1);
        }
        return true;
      }

      if (find_near(fs, edir)) return true;

      return find_gap(fs, edir);
    };

    std::function<bool(State & fs)> find_next = find_next_no_gap;
    if (max_gap_ > 0) find_next = find_next_gap;

    auto search_no_cr = [&](State state, char) {
      // set map
      plsmap[state.idx] = seg.id;
      // update line point list
      this->points_.push_back(PT(state.x, state.y));

      while (find_next(state)) {
        // set map
        plsmap[state.idx] = seg.id;
        this->points_.push_back(PT(state.x, state.y));
      }
    };

    auto search_cr = [&](State search_state, char ls_dir) {
      State fs, fs2;

      // find next pixels
      while (find_next(fs = search_state)) {
        int diff_s = abs_diffmap[static_cast<int>(ls_dir - search_state.dir)];
        int diff_fs = abs_diffmap[static_cast<int>(ls_dir - fs.dir)];
        //                         |
        // check for hard corner  _|
        if (diff_s > 1 && diff_fs > 1) return;

        //                           |        |
        //                           |       /
        // check for hard corners  _/   or _|
        if (find_next(fs2 = fs)) {
          int diff_fs2 = abs_diffmap[static_cast<int>(ls_dir - fs2.dir)];

          if (diff_fs > 1 && diff_fs2 > 1) {
            //      \     _
            //       \     \    __
            // case _/    _/    _/
            if (diff_fs > 2 && diff_fs2 > 2) {
              // add search_state to current pattern
              // set map and add point to list
              plsmap[search_state.idx] = seg.id;
              this->points_.push_back(PT(search_state.x, search_state.y));
            }
            return;
          }
          //                            |
          //                           /
          // check for hard corner   _|
          if (diff_s == 2 && search_state.dir == fs2.dir) return;
        }
        // set map
        plsmap[search_state.idx] = seg.id;
        this->points_.push_back(PT(search_state.x, search_state.y));
        ls_dir = search_state.dir;
        search_state = fs;
      }

      // set map
      plsmap[search_state.idx] = seg.id;
      this->points_.push_back(PT(search_state.x, search_state.y));
    };


    std::function<void(State s, char)> search = search_no_cr;
    if (flags_ & CC_CORNER_RULE) search = search_cr;


    for_each(seeds_.begin(), seeds_.end(), [&](const PT& p) {
      int idx = static_cast<int>(point2Index(PT(getX(p), getY(p)), cols_));
      if (plsmap[idx]) return;

      State initial_state = State(idx, getX(p), getY(p), pemap[idx]);
      State ls = initial_state;
      seg.p_beg = this->points_.size();
      // CV_Assert(initial_state.x != 83 | initial_state.y != 61);

      // rv: <---, fw: --->
      const short(*rvdmap)[4] = dmap + 4;
      const short(*fwdmap)[4] = dmap;

      switch (initial_state.dir) {
        case 0:  // <---x---> : fw: <---, rv: --->
        case 7:
          // just switch fw with rv
          fwdmap = rvdmap;
          rvdmap = dmap;
          [[fallthrough]];
        case 3:  // <---x---> : rv: <---, fw: --->
        case 4: {
          State ps = s;
          pdmap = fwdmap;
          // check for fw points
          if (!find_next_no_check(ps)) {
            // no fw points found, jump to rv search
            goto rv;
          }

          pdmap = rvdmap;
          // check for rv points
          if (!find_next_no_check(ls)) {
            // if no rv points found, jump to normal fw search
            goto fw;
          }

          // check for rv points
          if (!find_next_no_check(ps = ls)) {
            s = ls;
            // if no rv points found, jump to normal fw search
            goto fw;
          }

          // do rv first
          plsmap[ls.idx] = seg.id;
          this->points_.push_back(PT(ls.x, ls.y));
          search(ps, ls.dir);

          // closed check
          if (this->points_.back() == PT(s.x, s.y)) {
            seg.reverse = fwdmap != dmap;
          } else {
            std::reverse(this->points_.begin() + static_cast<std::ptrdiff_t>(seg.p_beg), this->points_.end());

            // do fw
            pdmap = fwdmap;
            s.dir = ls.dir;
            search(s, ps.dir);

            seg.reverse = fwdmap == dmap;
          }
          seg.p_end = this->points_.size();
        } break;
        case 1:
        case 2:  // rv
        rv:
          pdmap = rvdmap;
          plsmap[s.idx] = seg.id;
          this->points_.push_back(PT(s.x, s.y));

          if (find_next_no_check(s)) search(s, ls.dir);

          seg.p_end = this->points_.size();
          seg.reverse = fwdmap != dmap;
          break;
        case 5:
        case 6:  // fw
        fw:
          pdmap = fwdmap;
          plsmap[s.idx] = seg.id;
          this->points_.push_back(PT(s.x, s.y));

          if (find_next_no_check(s)) search(s, ls.dir);

          seg.p_end = this->points_.size();
          seg.reverse = fwdmap == dmap;
          break;
      }

      if (seg.p_end - seg.p_beg >= static_cast<size_t>(min_pix_)) {
        segments_.push_back(seg);
      }
      ++seg.id;
    });
  }

  void splitSeg() {
    typedef typename PointVector::iterator LinePointIter;
    lineData_.clear();
    lineData_.reserve(static_cast<size_t>(segments_.capacity()));
    const LineData* pseg = 0;
    LinePointIter lpbeg = points_.begin();

    // recursive solution
    std::function<void(size_t, size_t)> search;
    search = [&](size_t sbeg, size_t send) {
      size_t s = send - sbeg;
      if (s < static_cast<size_t>(min_pix_)) return;

      if (static_cast<FT>(s) < 2 * err_dist_) {
        lineData_.push_back(LineData(pseg->id, sbeg, send, pseg->reverse, &this->points_));
        return;
      }

      LinePointIter beg = lpbeg + static_cast<std::ptrdiff_t>(sbeg),
                    end = lpbeg + static_cast<std::ptrdiff_t>(send) - 1;
      size_t max_point = sbeg;

      const PT& first = *beg;
      bool no_gap = true;
      // get direction of line
      PT n = (*end - first), a;
      // get normal
      int max_h = getX(n), max_count = 0, h;
      getX(n) = -getY(n);
      getY(n) = max_h;
      max_h = 0;
      ++beg;

      for (; beg != end; ++beg) {
        a = *beg - first;
        if ((h = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
          max_h = h;
          max_point = static_cast<size_t>(beg - lpbeg);
          max_count = 0;
          no_gap = true;
          continue;
        }
        if (h == max_h && no_gap)
          ++max_count;
        else
          no_gap = false;
      }

      if (max_h > err_dist_ * std::sqrt(squaredNorm(n))) {
        size_t max_end = max_point;

        if (max_count != 0) {
          // corner / center check
          max_end += static_cast<size_t>(max_count);
          if (sbeg + 3 > max_point) {
            max_end = max_point;
          } else if (max_end + 4 > send) {
            max_point = max_end;
          } else {
            PT tmp = this->points_[max_end] - this->points_[max_point];
            float mdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
            tmp = this->points_[max_point - 1] - this->points_[max_point - 3];
            float bdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
            tmp = this->points_[max_end + 3] - this->points_[max_end + 1];
            float edir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
            int mbdiff = static_cast<int>(mdir - bdir + 360) % 180;
            int mediff = static_cast<int>(mdir - edir + 360) % 180;

            if (mbdiff < 45 && mediff < 45) {
              max_point += static_cast<size_t>(max_count / 2);
              max_end = max_point + static_cast<size_t>(max_count % 2);
            } else if (mbdiff > mediff) {
              max_end = max_point + (max_count == 1 && mediff > 45);
            } else {
              max_point = max_end - (max_count == 1 && mbdiff > 45);
            }
          }
        }

        search(sbeg, max_point + 1);
        search(max_end, send);
      } else
        lineData_.push_back(LineData(pseg->id, sbeg, send, pseg->reverse, &this->points_));
    };

    for_each(segments_.begin(), segments_.end(), [&](const LineData& seg) {
      pseg = &seg;
      search(seg.p_beg, seg.p_end);
    });
  }

  void computeLines() {
    lineSegments_.reserve(static_cast<size_t>(lineData_.size()));
    typedef typename PointVector::const_iterator const_iter;

    for_each(lineData_.begin(), lineData_.end(), [&](LineData& ldata) {
      if (ldata.size() < static_cast<size_t>(min_pix_)) return;

      const_iter beg = ldata.begin(), end = ldata.end();

      const PT& first = ldata.reverse ? *beg : *(end - 1);
      const PT& last = ldata.reverse ? *(end - 1) : *beg;

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
