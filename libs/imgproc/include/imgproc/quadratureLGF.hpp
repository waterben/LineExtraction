#ifndef _QUADRATURELGF_HPP_
#define _QUADRATURELGF_HPP_
#ifdef __cplusplus

#include <opencv2/core/core.hpp>
#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {
    //! Spherical Log Gabor quadraturte filter operating in freqency space
    template<class IT, class FT, template<typename, typename> class P = Polar>
    class QuadratureLGF : public Quadrature<IT, FT, FT, FT, FT> {

    public:
        static void createFilter(cv::Mat_<FT> &e, cv::Mat_<std::complex<FT>> &o, int rows, int cols, FT wavelength, FT sigmaOnf, FT cutoff = static_cast<FT>(0.45), int n = 15) {
            cv::Mat u1, u2, r;
            filterGrid<FT>(rows, cols, r, u1, u2);
            r.at<FT>(0, 0) = 1;
            divideComplex<FT>(merge<FT>(-u2, u1), r, o);
            
            FT lsOnf = std::log(sigmaOnf);
            lsOnf = -1 / (2 * lsOnf * lsOnf);
            
            cv::log(r * wavelength, u1);
            cv::exp(u1.mul(u1) * lsOnf, e);
            cv::multiply(e, lowpassFilter<FT>(rows, cols, cutoff, n), e);
            e.template at<FT>(0, 0) = 0;
        }
    
    protected:
        int rows_, cols_, rows_ext_, cols_ext_;
        FT waveL_, sigmaOnf_;

        mutable cv::Mat_<FT> dir_, phase_, energy_, o_, ox_, oy_, e_;
        mutable bool dir_done_, phase_done_, odd_done_, energy_done_;
        Range<FT> energyRange_, oddRange_, evenRange_;


        cv::Mat_<std::complex<FT>> H_;
        cv::Mat_<FT> lgf_;

        void max_response() {

            cv::Mat_<IT> tmp(128, 128);
            tmp.setTo(this->intRange_.lower);
            tmp.colRange(0, 64).setTo(this->intRange_.upper);
            // force update
            updateFilter(tmp);
            process(tmp);
            double vmin, vmax;
            cv::minMaxIdx(energy(), &vmin, &vmax);
            energyRange_.upper = static_cast<FT>(vmax);
            cv::minMaxIdx(odd(), &vmin, &vmax);
            oddRange_.upper = static_cast<FT>(vmax);
            cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
            evenRange_.upper = static_cast<FT>(vmax);
            evenRange_.lower = -evenRange_.upper;
        }

        void updateFilter(const cv::Mat img, bool force = true) {
            // if force is of, only renew if image size has changed
            if (!force && img.rows == rows_ && img.cols == cols_)
                return;

            rows_ = img.rows;
            cols_ = img.cols;
            rows_ext_ = cv::getOptimalDFTSize(img.rows);
            cols_ext_ = cv::getOptimalDFTSize(img.cols);

            createFilter(lgf_, H_, rows_ext_, cols_ext_, waveL_, sigmaOnf_);
        }

        void init() {
            this->add("grad_waveLength", std::bind(&QuadratureLGF<IT,FT,P>::valueWaveLength, this, std::placeholders::_1), "Wave length of scale filter.");
            this->add("grad_sigmaOnf", std::bind(&QuadratureLGF<IT, FT, P>::valueSigmaOnf, this, std::placeholders::_1), "Ratio of the standard deviation of the Gaussian describing the log Gabor filter's transfer function in the frequency domain to the filter center frequency.");
            
            max_response();
        }


	public:
        typedef IT img_type;
        typedef FT grad_type;
        typedef FT mag_type;
        typedef FT energy_type;
        typedef FT dir_type;
        typedef FT phase_type;

        typedef Range<IT> IntensityRange;
        typedef Range<FT> GradientRange;
        typedef Range<FT> MagnitudeRange;
        typedef Range<FT> EnergyRange;
        typedef Range<FT> DirectionRange;
        typedef Range<FT> PhaseRange;

        QuadratureLGF(FT waveLength = 5, FT sigmaOnf = static_cast<FT>(0.55), IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max()) 
            : Quadrature<IT,FT,FT,FT,FT>(int_lower, int_upper), rows_(0), cols_(0), rows_ext_(0), cols_ext_(0), waveL_(waveLength), sigmaOnf_(sigmaOnf)  {
            init();
        }

        QuadratureLGF(const ValueManager::NameValueVector &options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper), rows_(0), cols_(0), rows_ext_(0), cols_ext_(0), waveL_(3), sigmaOnf_(static_cast<FT>(0.55)) {
            init();
            value(options);
        }

        QuadratureLGF(ValueManager::InitializerList options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper), rows_(0), cols_(0), rows_ext_(0), cols_ext_(0), waveL_(3), sigmaOnf_(static_cast<FT>(0.55)) {
            init();
            value(options);
        }

        Value valueWaveLength(const Value &w = Value::NAV()) { if (w.type()) waveLength(w);  return waveL_; }

        FT waveLength() const { return waveL_; }

        void waveLength(FT w) {
            if (w == waveL_ || w <= 0)
                return;
            waveL_ = w;
            max_response();
        }

        Value valueSigmaOnf(const Value &so = Value::NAV()) { if (so.type()) sigmaOnf(so); return sigmaOnf_; }

        FT sigmaOnf() const { return sigmaOnf_; }

        void sigmaOnf(FT s) {
            if (s == sigmaOnf_ || s <= 0)
                return;
            sigmaOnf_ = s;
            max_response();
        }

        //! process gradient
        void process(const cv::Mat& img) {
            
            cv::Mat_<std::complex<FT>> IM, tmpv;

            dir_done_ = false;
            odd_done_ = false;
            phase_done_ = false;
            energy_done_ = false;
            
            updateFilter(img, false);

            cv::Mat_<FT> src;
            if (img.type() != cv::DataType<FT>::type)
                img.convertTo(src, cv::DataType<FT>::type);
            else
                src = img;
            
            if (cols_ < cols_ext_ || rows_ < rows_ext_)
                cv::copyMakeBorder(src, src, 0, rows_ext_ - rows_, 0, cols_ext_ - cols_, cv::BORDER_REPLICATE);

#ifdef USE_PERIODIC_FFT //slower, but removes artifacts
            IM = perfft2<FT>(src);
#else
            IM = fft2<FT>(src);
#endif

            tmpv = multiply(IM, lgf_);
            e_ = real(ifft2<FT>(tmpv));
            tmpv = ifft2(multiply(tmpv, H_));

            if (cols_ < cols_ext_ || rows_ < rows_ext_) {
                tmpv.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
                e_.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
            }

            splitT(tmpv, ox_, oy_);
        }

        //! get even response
        cv::Mat even() const {
            return e_;
        }

        GradientRange evenRange() const {
            return evenRange_;
        }

        //! get x,y odd responses
        void odd(cv::Mat& ox, cv::Mat& oy) const {
            ox = ox_; oy = oy_;
        }

        //! test if odd is computed
        inline bool isOddDone() const {
            return odd_done_;
        }

        //! get odd repsonse
        cv::Mat odd() const {
            if (!odd_done_) {
                Polar<FT, FT>::magnitude(ox_, oy_, o_);
                odd_done_ = true;
            }
            return o_;
        }

        //! get odd response range
        MagnitudeRange oddRange() const {
            return oddRange_;
        }

        
        //! test if direction is computed
        inline bool isDirectionDone() const {
            return dir_done_;
        }

        //! get direction
        cv::Mat direction() const {
            if (!dir_done_) {
                Polar<FT, FT>::phase(ox_, oy_, dir_);
                dir_done_ = true;
            }
            return dir_;
        }

        //! get direction range ([-PI,PI], [0,2PI] or [0,360])
        DirectionRange directionRange() const {
            return Polar<FT, FT>::range();
        }

        //! test if energy is computed
        inline bool isEnergyDone() const {
            return energy_done_;
        }

        //! get energy
        inline  cv::Mat energy() const {
            if (!energy_done_) {
                P<FT, FT>::magnitude(odd(), e_, energy_);
                energy_done_ = true;
            }
            return energy_;
        }

        //! get odd response range
        EnergyRange energyRange() const {
            return energyRange_;
        }


        //! test if phase is computed
        inline bool isPhaseDone() const {
            return phase_done_;
        }

        //! get phase
        cv::Mat phase() const {
            if (!phase_done_) {
                Polar<FT, FT>::phase(odd(), e_, phase_);
                phase_done_ = true;
            }
            return phase_;
        }

        //! get direction range ([-PI,PI], [0,2PI] or [0,360])
        PhaseRange phaseRange() const {
            return Polar<FT, FT>::range();
        }

        //! get name of direction operator
        std::string name() const {
            return "quadratureLGF";
        }

        using ValueManager::values;
        using ValueManager::valuePair;
        using ValueManager::value;
        using Quadrature<IT, FT, FT, FT, FT>::intensityRange;
        using Quadrature<IT, FT, FT, FT, FT>::results;
        using Quadrature<IT, FT, FT, FT, FT>::evenThreshold;
        using Quadrature<IT, FT, FT, FT, FT>::oddThreshold;
        using Quadrature<IT, FT, FT, FT, FT>::oddx;
        using Quadrature<IT, FT, FT, FT, FT>::oddy;
        using Quadrature<IT, FT, FT, FT, FT>::oddGradRange;
        using Quadrature<IT, FT, FT, FT, FT>::normType;
        using Quadrature<IT, FT, FT, FT, FT>::energyThreshold;

	};

}

#endif
#endif
