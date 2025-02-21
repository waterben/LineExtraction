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
// C by Benjamin Wassermann
//M*/

#ifndef _THRESHOLD_HPP_
#define _THRESHOLD_HPP_
#ifdef __cplusplus

#include <edge/otsu.hpp>

namespace lsfm {
    
    template<class IT>
    class Threshold {
    protected:
        Threshold() {}
    public:
        typedef IT img_type;

        virtual ~Threshold() {}

        virtual cv::Mat_<IT> process(const cv::Mat &img) = 0;

        virtual std::string name() const = 0;
    };

    //! Global threshold class
    template<class IT, class E = ThresholdOtsu<IT,512,float>>
    class GlobalThreshold : public Threshold<IT> {
        E th_;
    public:
        typedef IT img_type;

        GlobalThreshold(IT rMax = std::numeric_limits<IT>::max()) : th_(rMax) {}

        
        //! Compute global threshold
        cv::Mat_<IT> process(const cv::Mat &mag) {
            cv::Mat_<IT> ret(mag.size());
            ret.setTo(th_.process(mag));
            return ret;
        }


        //! Get name of threshold method
        std::string name() const {
            return "global " + E::name();
        }

    };

    //! Local threshold class using fixed window
    template<class IT, class E = ThresholdOtsu<IT, 512, float>>
    class LocalThreshold : public Threshold<IT> {
    protected:
        E th_;
        int wx_, wy_;
        bool as_;
    public:
        typedef IT img_type;

        LocalThreshold(int win_size_x, int win_size_y, bool auto_adapt_size = true, IT rMax = std::numeric_limits<IT>::max()) : 
            th_(rMax), wx_(win_size_x), wy_(win_size_y), as_(auto_adapt_size) {}


        //! Compute local threshold
        cv::Mat_<IT> process(const cv::Mat &mag) {
            cv::Size s = mag.size();
            cv::Mat_<IT> ret(s);

            int wx = wx_, wy = wy_;
            int tilesx = s.width / wx, tilesy = s.height / wy;
            int rx = s.width % wx, ry = s.height % wy;
            
            if (as_) {
                wx += rx / tilesx;
                rx = rx % tilesx;

                wy += ry / tilesy;
                ry = ry % tilesy;

                int starty = 0;
                for (int y = 0; y != tilesy; ++y) {
                    int endy = starty + wy + (ry > y ? 1 : 0);
                    cv::Mat row_ret = ret.rowRange(starty, endy), row_mag = mag.rowRange(starty, endy);
                    starty = endy;
                    int startx = 0;
                    for (int x = 0; x != tilesx; ++x) {
                        int endx = startx + wx + (rx > x ? 1 : 0);
                        row_ret.colRange(startx, endx).setTo(th_.process(row_mag.colRange(startx, endx)));
                        startx = endx;
                    }
                }
            }
            else {
                int starty = 0;
                if (ry)
                    ++tilesy;
                for (int y = 0; y != tilesy; ++y) {
                    int endy = std::min(s.height,starty + wy);
                    cv::Mat row_ret = ret.rowRange(starty, endy), row_mag = mag.rowRange(starty, endy);
                    starty = endy;
                    int startx = 0;
                    for (int x = 0; x != tilesx; ++x) {
                        int endx = startx + wx;
                        row_ret.colRange(startx, endx).setTo(th_.process(row_mag.colRange(startx, endx)));
                        startx = endx;
                    }
                    if (rx)
                        row_ret.colRange(startx, row_ret.cols).setTo(th_.process(row_mag.colRange(startx, row_mag.cols)));
                }
            }

            return ret;
        }


        //! Get name of threshold method
        std::string name() const {
            return "local window " + E::name();
        }

    };

    //! Local threshold class using tiles
    template<class IT, class E = ThresholdOtsu<IT, 512, float>>
    class LocalThresholdTiles : public LocalThreshold<IT,E> {
        int tx_, ty_;
    public:
        typedef IT img_type;

        LocalThresholdTiles(int tiles_x, int tiles_y, IT rMax = std::numeric_limits<IT>::max()) :
            LocalThreshold<IT,E>(0, 0, true, rMax), tx_(tiles_x), ty_(tiles_y)  {}


        //! Compute local threshold
        cv::Mat_<IT> process(const cv::Mat &mag) {
            cv::Size s = mag.size();
            this->wx_ = s.width / tx_;
            this->wy_ = s.height / ty_;
            return LocalThreshold<IT, E>::process(mag);
        }


        //! Get name of threshold method
        std::string name() const {
            return "local tiles " + E::name();
        }

    };

    //! Dynamic threshold class using radius to define sliding window size
    template<class IT, class E = ThresholdOtsu<IT, 512, float>>
    class DynamicThreshold : public Threshold<IT> {
    protected:
        E th_;
        int r_;
    public:
        typedef IT img_type;

        DynamicThreshold(int radius, IT rMax = std::numeric_limits<IT>::max()) :
            th_(rMax), r_(radius) {}


        //! Compute dynamic threshold
        cv::Mat_<IT> process(const cv::Mat &mag) {
            cv::Size s = mag.size();
            cv::Mat_<IT> ret(s);

            // compute without border checks
            int endy = s.height - r_;
            int endx = s.width - r_;

            // first compute inner
            for (int y = r_; y != endy; ++y) {
                cv::Mat win_mag = mag.rowRange(y - r_, y + r_).colRange(0, r_ + r_);
                for (int x = r_; x != endx; ++x) {
                    ret(y,x) = th_.process(win_mag);
                    win_mag.adjustROI(0, 0, -1, 1);
                }
            }

            // compute top rows
            for (int y = 0; y != r_; ++y) {
                cv::Mat win_mag = mag.rowRange(0, y + r_).colRange(0, r_ + r_);
                for (int x = r_; x != endx; ++x) {
                    ret(y, x) = th_.process(win_mag);
                    win_mag.adjustROI(0, 0, -1, 1);
                }
            }

            // compute bottom rows
            for (int y = s.height - r_; y != s.height; ++y) {
                cv::Mat win_mag = mag.rowRange(y - r_, s.height).colRange(0, r_ + r_);
                for (int x = r_; x != endx; ++x) {
                    ret(y, x) = th_.process(win_mag);
                    win_mag.adjustROI(0, 0, -1, 1);
                }
            }

            // compute left cols
            for (int y = r_; y != endy; ++y) {
                cv::Mat row_mag = mag.rowRange(y - r_, y + r_);
                for (int x = 0; x != r_; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(0, x + r_));
                }
            }

            // compute right cols
            for (int y = r_; y != endy; ++y) {
                cv::Mat row_mag = mag.rowRange(y - r_, y + r_);
                for (int x = s.width - r_; x != s.width; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(x - r_, s.width));
                }
            }

            // compute top left
            for (int y = 0; y != r_; ++y) {
                cv::Mat row_mag = mag.rowRange(0, y + r_);
                for (int x = 0; x != r_; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(0, x + r_));
                }
            }

            // compute top right
            for (int y = 0; y != r_; ++y) {
                cv::Mat row_mag = mag.rowRange(0, y + r_);
                for (int x = s.width - r_; x != s.width; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(x - r_, s.width));
                }
            }

            // compute bottom left
            for (int y = s.height - r_; y != s.height; ++y) {
                cv::Mat row_mag = mag.rowRange(y - r_, s.height);
                for (int x = 0; x != r_; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(0, x + r_));
                }
            }

            // compute bottom right
            for (int y = s.height - r_; y != s.height; ++y) {
                cv::Mat row_mag = mag.rowRange(y - r_, s.height);
                for (int x = s.width - r_; x != s.width; ++x) {
                    ret(y, x) = th_.process(row_mag.colRange(x - r_, s.width));
                }
            }

            return ret;
        }

        //! Get name of threshold method
        std::string name() const {
            return "dynamic " + E::name();
        }

    };
    
}
#endif
#endif
