#ifndef _PHASE_CONG_HPP_
#define _PHASE_CONG_HPP_

#include <opencv2/core/core.hpp>

// Forward declarations
struct emxArray_real_T;
struct emxArray_creal_T;

namespace lsfm {

	// best to use image size with power of 2 because fft is used
	class PhaseCong {
        emxArray_real_T *lgf;
        emxArray_creal_T *H;
    protected:
        int rows_, cols_, nscale_, sizep2_;
        double minW_, mult_, sigmaOnf_;

        void updateFilter();

	public:
		// init object and preinit filters
        PhaseCong(int ns = 4, double mw = 3, double ml = 2.1, double sig = 0.55);
		~PhaseCong();
		// process phase congruency with preinited filters -> image size must match filter size...
        void run(const cv::Mat &img, cv::Mat &e, cv::Mat &ox, cv::Mat &oy, cv::Mat &energy, cv::Mat &pc, double k = 3, double cutOff = 0.5, double g = 10, double deviationGain = 1.5, double noiseMethod = -1);
	};

}


#endif

