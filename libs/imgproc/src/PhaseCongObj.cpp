#include <imgproc/impl/PhaseCong.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "logGaborFilter_types.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "logGaborFilter_terminate.h"
#include "logGaborFilter_emxAPI.h"
#include "logGaborFilter_emxutil.h"
#include "logGaborFilter_initialize.h"

void doInit() {
	static bool init = false;
	if (!init) {
		init = true;
		logGaborFilter_initialize();
	}
}

inline uint nextPowerOf2(uint n)
{
	/*
	* Below indicates passed no is a power of 2, so return the same.
	*/
	if ((n == 0) || !(n & (n - 1))) {
		return (n);
	}

	while (n & (n - 1)) {
		n = n & (n - 1);
	}

	n = n << 1;
	return n;
}

inline void prepare(const cv::Mat &img, int sizep2, cv::Mat &store, cv::Mat &e, cv::Mat &ox, cv::Mat &oy, cv::Mat &energy, cv::Mat &pc) {
	cv::copyMakeBorder(img, store, 0, sizep2 - img.rows, 0, sizep2 - img.cols, cv::BORDER_REPLICATE);
	if (store.type() != CV_64F)
		store.convertTo(store, CV_64F);
	e.create(sizep2, sizep2, CV_64F);
	ox.create(sizep2, sizep2, CV_64F);
    oy.create(sizep2, sizep2, CV_64F);
    energy.create(sizep2, sizep2, CV_64F);
    pc.create(sizep2, sizep2, CV_64F);
}

namespace lsfm {

	
    PhaseCong::PhaseCong(int ns, double mw, double ml, double sig) :
        rows_(0), cols_(0), nscale_(ns), minW_(mw), mult_(ml), sigmaOnf_(sig), sizep2_(0), lgf(0), H(0) {
		
		doInit();
	}

	PhaseCong::~PhaseCong() {
		emxFree_creal_T(&H);
		emxFree_real_T(&lgf);
	}

    void PhaseCong::updateFilter() {
        emxFree_creal_T(&H);
        emxFree_real_T(&lgf);

        emxInit_real_T2(&lgf, 3);
        emxInit_creal_T(&H, 2);

        logGaborFilter(sizep2_, sizep2_, nscale_, minW_, mult_, sigmaOnf_, lgf, H);
    }
	
    void PhaseCong::run(const cv::Mat &img, cv::Mat &e, cv::Mat &ox, cv::Mat &oy, cv::Mat &energy, cv::Mat &pc, double k, double cutOff, double g, double deviationGain, double noiseMethod) {
        if (img.rows != rows_ || img.cols != cols_) {
            rows_ = img.rows;
            cols_ = img.cols;
            sizep2_ = std::max(nextPowerOf2(rows_), nextPowerOf2(cols_));
            updateFilter();
        }
			
		cv::Mat store;
        prepare(img, sizep2_, store, e, ox, oy, energy,pc);

		emxArray_real_T*  in = emxCreateWrapper_real_T(store.ptr<double>(), store.rows, store.cols);
		emxArray_real_T*  inE = emxCreateWrapper_real_T(e.ptr<double>(), e.rows, e.cols);
		emxArray_real_T*  inOx = emxCreateWrapper_real_T(ox.ptr<double>(), ox.rows, ox.cols);
        emxArray_real_T*  inOy = emxCreateWrapper_real_T(oy.ptr<double>(), oy.rows, oy.cols);
        emxArray_real_T*  inEnergy = emxCreateWrapper_real_T(energy.ptr<double>(), energy.rows, energy.cols);
        emxArray_real_T*  inPc = emxCreateWrapper_real_T(pc.ptr<double>(), pc.rows, pc.cols);

        phasecong(in, lgf, H, mult_, k, cutOff, g, deviationGain, noiseMethod, inE, inOy, inOx, inEnergy, inPc);
        if (rows_ != sizep2_) {
            e = e.rowRange(0, rows_);
            ox = ox.rowRange(0, rows_);
            oy = oy.rowRange(0, rows_);
            energy = energy.rowRange(0, rows_);
            pc = pc.rowRange(0, rows_);
		}
        if (cols_ != sizep2_) {
            e = e.colRange(0, cols_);
            ox = ox.colRange(0, cols_);
            oy = oy.colRange(0, cols_);
            energy = energy.colRange(0, cols_);
            pc = pc.colRange(0, cols_);
		}
	}

}


