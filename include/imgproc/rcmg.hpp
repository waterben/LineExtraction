/***************************************************************************
 * The Robust Colour Morphological Gradient (RCMG) for edge detection      *
 * in multichannel images                                                  *
 *                                                                         *
 * Adrian N. Evans                                                         *
 * University of Bath                                                      *
 * July 2007                                                               *
 * Copyright 2005-2007.  All rights reserved.                              *
 *                                                                         *
 * The RCMG find the median centered difference for a given window size    *
 * by rejecting the s pairs of pixels that are furthest apart. For full    *
 * details of its operation see A.N. Evans and X.U. Liu, A Morphological   *
 * Gradient Approach to Color Edge Detection, IEEE Transactions on Image   *
 * Processing, 15(6), pp. 1454-1463, June 2006.                            *
 *																		   *
 * This implementation uses the L2 norm                                    *
 *																		   *
 ***************************************************************************/

#ifndef _RCMG_HPP_
#define _RCMG_HPP_
#ifdef __cplusplus

#include <opencv2/core/core.hpp>
#include <imgproc/gradient.hpp>
#include <vector>

namespace lsfm {

    template<class IT = uchar, int channels = 3, class GT = short, class MT = int, class DT = float, class DO = Direction<GT, DT>>
    class RCMGradient : public Gradient<IT, GT, MT, DT> {

        int mask_, s_, m2_, norm_;

        cv::Mat_<MT> mag_;
        cv::Mat_<GT> gx_, gy_;

        mutable cv::Mat_<DT> dir_;
        mutable bool dir_done_;


        // vectors
        std::vector<int> locations;
        std::vector<char> not_removed;
        std::vector<cv::Vec<IT,channels>> window;
        MT max_dist, max_dist2;
        std::vector<MT> max_dists, copyof_max_dists;
        cv::Mat_<MT> dists;

	public:
		typedef IT img_type;
		typedef GT grad_type;
		typedef MT mag_type;
		typedef DT dir_type;

        typedef Range<GT> GradientRange;
        typedef Range<MT> MagnitudeRange;
        typedef Range<DT> DirectionRange;

        //! mask for window size and s for vector rejection (noise reduction)
        RCMGradient(int mask = 3, int s = 0, int cn = cv::NORM_L2SQR, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : Gradient<IT, GT, MT, DT>(int_lower, int_upper), mask_(mask), s_(s), norm_(cn) {

            this->add("grad_mask_size", std::bind(&RCMGradient<IT,channels,GT,MT,DT,DO>::maskSize,this,std::placeholders::_1),"Window size for differences.");
            this->add("grad_rejection", std::bind(&RCMGradient<IT,channels,GT,MT,DT,DO>::rejection,this,std::placeholders::_1),"Vector rejection for noise compensation.");
            this->add("grad_color_norm", std::bind(&RCMGradient<IT,channels,GT,MT,DT,DO>::colorNorm,this,std::placeholders::_1),"Norm type: L1 or L2.");
            updateMask(mask);

        }

        Value maskSize(const Value &ms = Value::NAV()) { if (ms.type()) {updateMask(ms); } return mask_;}
        Value rejection(const Value &r = Value::NAV()) { if (r.type()) s_ = r;  return s_;}
        Value colorNorm(const Value &nt = Value::NAV()) { if (nt.type()) norm_ = nt;  return norm_;}


		//! process gradient
        void process(const cv::Mat& img) {
            dir_done_ = false;
            do_process(img);
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
            gx = gx_; gy = gy_;
        }

        //! get x derivative
        cv::Mat gx() const {
            return gx_;
        }

        //! get y derivative
        cv::Mat gy() const {
            return gy_;
        }

		GradientRange gradientRange() const {
			return GradientRange(-mask_, mask_);
		}

        //! get magnitude
        cv::Mat magnitude() const {
            return mag_;
        }

        //! test if direction is computed
        inline bool isDirectionDone() const {
            return dir_done_;
        }

        //! get direction
        cv::Mat direction() const {
            if (!dir_done_) {
                DO::process(gx_, gy_, dir_);
                dir_done_ = true;
            }
            return dir_;
        }

        //! get direction range ([-PI,PI], [0,2PI] or [0,360])
        DirectionRange directionRange() const {
            return DO::range();
        }

        MagnitudeRange magnitudeRange() const {
            return MagnitudeRange(0,static_cast<MT>(norm(cv::Vec<IT,channels>::all(this->intRange_.upper),norm_)));
        }


        //! get name of gradient operator
        inline std::string name() const {
            return "rcmg";
        }

	private:
        template<typename T, int m, int n> 
        static inline double norm(const cv::Matx<T, m, n>& M, int normType)
        {
            return normType == cv::NORM_INF ? (double)cv::normInf<T, typename cv::DataType<T>::work_type>(M.val, m*n) :
                normType == cv::NORM_L1 ? (double)cv::normL1<T, typename cv::DataType<T>::work_type>(M.val, m*n) :
                normType == cv::NORM_L2SQR ? (double)cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m*n) :
                std::sqrt((double)cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m*n));
        }

        template<typename T, int m, int n>
        static inline double norm(const cv::Matx<T, m, n>& A, const cv::Matx<T, m, n>& B, int normType)
        {
            return normType == cv::NORM_INF ? (double)cv::normInf<T, typename cv::DataType<T>::work_type>(A.val, B.val, m*n) :
                normType == cv::NORM_L1 ? (double)cv::normL1<T, typename cv::DataType<T>::work_type>(A.val, B.val, m*n) :
                normType == cv::NORM_L2SQR ? (double)cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(A.val, B.val, m*n) :
                std::sqrt((double)cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(A.val, B.val, m*n));
        }

        void updateMask(int m) {
            if (m < 3)
                m = 3;

            mask_ = m;
            int m2 = m*m;

            max_dists.resize(m2);
            locations.resize(m2);
            not_removed.resize(m2);
            window.resize(m2);
            dists.create(m2,m2);
        }

        void norms()
		{
            int i, j, location;
            MT n, max_norm;

            for (i = 0; i < dists.rows - 1; ++i) {
                max_norm = static_cast<MT>(-1);
                for (j = i + 1; j < dists.rows; ++j) {
                    n = static_cast<MT>(norm(window[i],window[j],norm_));
					if (n > max_norm) {
						max_norm = n;
						location = j;
					}
                    dists(i,j) = n;
				}
				max_dists[i] = max_norm;
				locations[i] = location;
			}
		}

        void updateNorms()
		{
            int i, j;
            MT n;

            for (i = 0; i < dists.rows - mask_ - 1; ++i) {		// copy existing values into new positions
                max_dists[i] = max_dists[i + mask_];		    // copy max dists and location and
                locations[i] = locations[i + mask_] - mask_;	// adjust by mask_ for move
                for (j = i + 1; j < dists.rows - mask_; ++j) {	// copy dists
                    dists(i,j) = dists(i + mask_,j + mask_);
				}

                for (j = i + 1; j < dists.rows - mask_; ++j) {	// copy dists
                    dists(i,j) = n = dists(i + mask_,j + mask_);
                    if (n > max_dists[i]) {
                        max_dists[i] = n;
						locations[i] = j;
					}
				}
			}

            for (i = dists.rows - mask_ - 1; i < dists.rows - 1; ++i)	// clear remaining max_dists
                max_dists[i] = static_cast<MT>(-1);

            for (i = 0; i < dists.rows - 1; ++i) {	// add new values and update max_dists
                for (j = cv::max(i + 1, mask_); j < dists.rows; ++j) {
                    dists(i,j) = n = static_cast<MT>(norm(window[i],window[j],norm_));
                    if (n > max_dists[i]) {
                        max_dists[i] = n;
						locations[i] = j;
					}
				}
			}
		}


        void do_process(const cv::Mat& img) {
            CV_Assert(img.channels() == channels);
            this->dir_done_ = false;

            // loops
            int imask, jmask, k, j, index, loop;

            int hs = (mask_ - 1) / 2;
            int m2 = mask_*mask_;
			//
            int v1, v2, location, pos;


			// magnitude, gx, gy
			// verwendung von mag gx gy aus gradient
            mag_.create(img.size());
            gx_.create(img.size());
            gy_.create(img.size());

            MT* pmag = this->mag_.template ptr<MT>();
            GT* pgx = this->gx_.template ptr<GT>();
            GT* pgy = this->gy_.template ptr<GT>();

            dists.setTo(-1);

			// iterate the image
            for (int i = hs; i < img.rows - hs - 1; ++i) {
                for (j = hs; j < img.cols - hs; ++j) {                    
					
					// grab the values for the window
					index = 0;
					// iterate the window
                    for (jmask = j - hs; jmask <= j + hs; ++jmask) {		//fill col-1 first
                        for (imask = i - hs; imask <= i + hs; ++imask) {
                            window[index] = img.at<cv::Vec<IT, channels>>(imask, jmask);
                            //window[index] /= std::numeric_limits<IT>::max();
							index++;
						}
					}

					// compute the norms
                    if (j == hs) //for first col
                        norms();
					else
                        updateNorms();


                    max_dist = static_cast<MT>(-1);
                    for (k = 0; k<m2 - 1; ++k) {			// *m2-1 as last row only -1s
						if (max_dists[k] > max_dist) {
							max_dist = max_dists[k];
                            v1 = k;						//Note: v1 and v2 also needed for direction
							v2 = locations[k];
						}
					}

                    if (s_ > 0) { // true if any pairs of vector going to be rejected
                        std::fill(not_removed.begin(),not_removed.end(),1);
                        copyof_max_dists = max_dists;
                        for (index = 1; index <= s_; ++index) {
							not_removed[v1] = 0;
							not_removed[v2] = 0;
                            max_dist = static_cast<MT>(-1);
                            for (k = 0; k<m2 - 1; ++k) {
								if (not_removed[k]) {
									if (copyof_max_dists[k] > max_dist) {
                                        if (not_removed[locations[k]]) {	// vector locations[k] not removed
											max_dist = copyof_max_dists[k];
											v1 = k;
											v2 = locations[k];
										}
                                        else {		// update max_dists
                                            max_dist2 = static_cast<MT>(-1);
                                            for (loop = k + 1; loop < m2; ++loop) {
                                                if (dists(k,loop) > max_dist2) {
													if (not_removed[loop]) {
                                                        max_dist2 = dists(k,loop);
														location = loop;
													}
												}
												copyof_max_dists[k] = max_dist2;
											}
                                            // repeat check for > max_dist2 with updated max_dists[k]
											if (copyof_max_dists[k] > max_dist) {
												max_dist = copyof_max_dists[k];
												v1 = k;
												v2 = location;
											}
										}
									}
								}
							}
						}
					}

                    pos = i * img.cols + j;
                    pmag[pos] = max_dist;
                    pgx[pos] = v2 / mask_ - v1 / mask_;
                    pgy[pos] = (v2 % mask_) - (v1 % mask_);
				}
			}
		}
	};

    template<class IT, class GT, class MT, class DT, class DO>
    class RCMGradient<IT,1,GT,MT,DT,DO> : public Gradient<IT, GT, MT, DT> {

        int mask_, s_, m2_, norm_;

        cv::Mat_<MT> mag_;
        cv::Mat_<GT> gx_, gy_;

        mutable cv::Mat_<DT> dir_;
        mutable bool dir_done_;


        // vectors
        std::vector<int> locations;
        std::vector<char> not_removed;
        std::vector<IT> window;
        MT max_dist, max_dist2;
        std::vector<MT> max_dists, copyof_max_dists;
        cv::Mat_<MT> dists;

    public:
        typedef IT img_type;
        typedef GT grad_type;
        typedef MT mag_type;
        typedef DT dir_type;

        typedef Range<GT> GradientRange;
        typedef Range<MT> MagnitudeRange;
        typedef Range<DT> DirectionRange;

        //! mask for window size and s for vector rejection (noise reduction)
        RCMGradient(int mask = 3, int s = 0, int nt = cv::NORM_L2SQR, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : Gradient<IT, GT, MT, DT>(int_lower, int_upper), mask_(mask), s_(s), norm_(nt) {

            this->add("grad_mask_size", std::bind(&RCMGradient<IT,1,GT,MT,DT,DO>::maskSize,this,std::placeholders::_1),"Window size for differences.");
            this->add("grad_rejection", std::bind(&RCMGradient<IT,1,GT,MT,DT,DO>::rejection,this,std::placeholders::_1),"Vector rejection for noise compensation.");
            this->add("grad_norm_type", std::bind(&RCMGradient<IT,1,GT,MT,DT,DO>::normType,this,std::placeholders::_1),"Norm type: L1 or L2.");
            updateMask(mask);
        }

        Value maskSize(const Value &ms = Value::NAV()) { if (ms.type()) {updateMask(ms); } return mask_;}
        Value rejection(const Value &r = Value::NAV()) { if (r.type()) s_ = r;  return s_;}
        Value normType(const Value &nt = Value::NAV()) { if (nt.type()) norm_ = nt;  return norm_;}


        //! process gradient
        void process(const cv::Mat& img) {
            dir_done_ = false;
            do_process(img);
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
            gx = gx_; gy = gy_;
        }

        //! get x derivative
        cv::Mat gx() const {
            return gx_;
        }

        //! get y derivative
        cv::Mat gy() const {
            return gy_;
        }

		GradientRange gradientRange() const {
			return GradientRange(-mask_, mask_);
		}

        //! get magnitude
        cv::Mat magnitude() const {
            return mag_;
        }

        //! test if direction is computed
        inline bool isDirectionDone() const {
            return dir_done_;
        }

        //! get direction
        cv::Mat direction() const {
            if (!dir_done_) {
                DO::process(gx_, gy_, dir_);
                dir_done_ = true;
            }
            return dir_;
        }

        //! get direction range ([-PI,PI], [0,2PI] or [0,360])
        DirectionRange directionRange() const {
            return DO::range();
        }

        MagnitudeRange magnitudeRange() const {
            return MagnitudeRange(0,static_cast<MT>(this->intRange_.upper));
        }


        //! get name of gradient operator
        inline std::string name() const {
            return "rcmg";
        }

    private:
        void updateMask(int m) {
            if (m < 3)
                m = 3;

            mask_ = m;
            int m2 = m*m;

            max_dists.resize(m2);
            locations.resize(m2);
            not_removed.resize(m2);
            window.resize(m2);
            dists.create(m2,m2);
        }

        void norms()
        {
            int i, j, location;
            MT norm, max_norm;

            for (i = 0; i < dists.rows - 1; ++i) {
                max_norm = -1;
                for (j = i + 1; j < dists.rows; ++j) {
                    norm = std::abs(static_cast<MT>(window[i])-window[j]);
                    if (norm > max_norm) {
                        max_norm = norm;
                        location = j;
                    }
                    dists(i,j) = norm;
                }
                max_dists[i] = max_norm;
                locations[i] = location;
            }
        }

        void updateNorms()
        {
            int i, j;
            MT n;

            for (i = 0; i < dists.rows - mask_ - 1; ++i) {		// copy existing values into new positions
                max_dists[i] = max_dists[i + mask_];		    // copy max dists and location and
                locations[i] = locations[i + mask_] - mask_;	// adjust by mask_ for move
                for (j = i + 1; j < dists.rows - mask_; ++j) {	// copy dists
                    dists(i,j) = dists(i + mask_,j + mask_);
                }

                for (j = i + 1; j < dists.rows - mask_; ++j) {	// copy dists
                    dists(i,j) = n = dists(i + mask_,j + mask_);
                    if (n > max_dists[i]) {
                        max_dists[i] = n;
                        locations[i] = j;
                    }
                }
            }

            for (i = dists.rows - mask_ - 1; i < dists.rows - 1; ++i)	// clear remaining max_dists
                max_dists[i] = -1;

            for (i = 0; i < dists.rows - 1; ++i) {	// add new values and update max_dists
                for (j = cv::max(i + 1, mask_); j < dists.rows; ++j) {
                    dists(i,j) = n = std::abs(static_cast<MT>(window[i])-window[j]);
                    if (n > max_dists[i]) {
                        max_dists[i] = n;
                        locations[i] = j;
                    }
                }
            }
        }


        void do_process(const cv::Mat& img) {
            CV_Assert(img.channels() == 1);
            this->dir_done_ = false;

            // loops
            int imask, jmask, k, j, index, loop;

            int hs = (mask_ - 1) / 2;
            int m2 = mask_*mask_;
            //
            int v1, v2, location, pos;


            // magnitude, gx, gy
            // verwendung von mag gx gy aus gradient
            mag_.create(img.size());
            gx_.create(img.size());
            gy_.create(img.size());

            MT* pmag = this->mag_.template ptr<MT>();
            GT* pgx = this->gx_.template ptr<GT>();
            GT* pgy = this->gy_.template ptr<GT>();

            dists.setTo(-1);

            // iterate the image
            for (int i = hs; i < img.rows - hs - 1; ++i) {
                for (j = hs; j < img.cols - hs; ++j) {

                    // grab the values for the window
                    index = 0;
                    // iterate the window
                    for (jmask = j - hs; jmask <= j + hs; ++jmask) {		//fill col-1 first
                        for (imask = i - hs; imask <= i + hs; ++imask) {
                            window[index] = img.at<IT>(imask, jmask);
                            index++;
                        }
                    }

                    // compute the norms
                    if (j == hs) //for first col
                        norms();
                    else
                        updateNorms();


                    max_dist = -1;
                    for (k = 0; k<m2 - 1; ++k) {			//m2-1 as last row only -1s
                        if (max_dists[k] > max_dist) {
                            max_dist = max_dists[k];
                            v1 = k;						//Note: v1 and v2 also needed for direction
                            v2 = locations[k];
                        }
                    }

                    if (s_ > 0) { // true if any pairs of vector going to be rejected
                        std::fill(not_removed.begin(),not_removed.end(),1);
                        copyof_max_dists = max_dists;
                        for (index = 1; index <= s_; ++index) {
                            not_removed[v1] = 0;
                            not_removed[v2] = 0;
                            max_dist = -1;
                            for (k = 0; k<m2 - 1; ++k) {
                                if (not_removed[k]) {
                                    if (copyof_max_dists[k] > max_dist) {
                                        if (not_removed[locations[k]]) {	// vector locations[k] not removed
                                            max_dist = copyof_max_dists[k];
                                            v1 = k;
                                            v2 = locations[k];
                                        }
                                        else {		// update max_dists
                                            max_dist2 = -1;
                                            for (loop = k + 1; loop < m2; ++loop) {
                                                if (dists(k,loop) > max_dist2) {
                                                    if (not_removed[loop]) {
                                                        max_dist2 = dists(k,loop);
                                                        location = loop;
                                                    }
                                                }
                                                copyof_max_dists[k] = max_dist2;
                                            }
                                            // repeat check for > max_dist2 with updated max_dists[k]
                                            if (copyof_max_dists[k] > max_dist) {
                                                max_dist = copyof_max_dists[k];
                                                v1 = k;
                                                v2 = location;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    pos = i * img.cols + j;
                    pmag[pos] = max_dist;
                    pgx[pos] = v2 / mask_ - v1 / mask_;
                    pgy[pos] = (v2 % mask_) - (v1 % mask_);
                }
            }
        }
    };
}

#endif
#endif
