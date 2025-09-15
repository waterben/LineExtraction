#ifndef _PC_SQF_HPP_
#define _PC_SQF_HPP_
#ifdef __cplusplus

#include <imgproc/phase_congruency.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <utility/matlab_helpers.hpp>
#include <utility/limit.hpp>

namespace lsfm {
    template<class IT = uchar, class FT = float, template<typename, typename> class P = Polar>
    class PCSqf : public PhaseCongruencyI<FT>, public QuadratureSF<IT, FT, P> {

        std::vector<cv::Mat_<std::complex<FT>>> fo_;
        std::vector<cv::Mat_<FT>> fe_;

        cv::Mat_<FT> weight, sumAn, zeros;

        mutable bool pc_done_;
        mutable cv::Mat_<FT> pc_;

        FT k_, g_, cutOff_, dGain_, nMethod_, eps_, T_;
        int nscale_;

        using QuadratureSF<IT, FT, P>::imgf_;

        using QuadratureSF<IT, FT, P>::o_;
        using QuadratureSF<IT, FT, P>::phase_;
        using QuadratureSF<IT, FT, P>::dir_;
        using QuadratureSF<IT, FT, P>::ox_;
        using QuadratureSF<IT, FT, P>::oy_;
        using QuadratureSF<IT, FT, P>::e_;
        using QuadratureSF<IT, FT, P>::energy_;
        using QuadratureSF<IT, FT, P>::rows_;
        using QuadratureSF<IT, FT, P>::cols_;
        using QuadratureSF<IT, FT, P>::rows_ext_;
        using QuadratureSF<IT, FT, P>::cols_ext_;

        using QuadratureSF<IT, FT, P>::kspacing_;
        using QuadratureSF<IT, FT, P>::scale_;
        using QuadratureSF<IT, FT, P>::muls_;

        using QuadratureSF<IT, FT, P>::odd_done_;
        using QuadratureSF<IT, FT, P>::energy_done_;
        using QuadratureSF<IT, FT, P>::dir_done_;
        using QuadratureSF<IT, FT, P>::phase_done_;
        using QuadratureSF<IT, FT, P>::even_done_;

        bool updateFilter(const cv::Mat img, bool force = true) {
            // if force is of, only renew if image size has changed
            if (!force && img.rows == rows_ && img.cols == cols_)
                return false;

            rows_ = img.rows;
            cols_ = img.cols;
            rows_ext_ = cv::getOptimalDFTSize(img.rows);
            cols_ext_ = cv::getOptimalDFTSize(img.cols);

            fe_.resize(nscale_);
            fo_.resize(nscale_);
            for (int i = 0; i != nscale_; ++i) {
                fe_[i] = ifftshift(QuadratureSF<IT, FT, P>::createFilter(rows_ext_, cols_ext_, kspacing_, scale_ + i, muls_, QuadratureSF<IT, FT, P>::dopf));
                fo_[i] = ifftshift(QuadratureSF<IT, FT, P>::createFilterC(rows_ext_, cols_ext_, kspacing_, scale_ + i, muls_, QuadratureSF<IT, FT, P>::docpf));
            }

            sumAn = cv::Mat_<FT>::zeros(rows_, cols_);
            ox_ = cv::Mat_<FT>::zeros(rows_, cols_);
            oy_ = cv::Mat_<FT>::zeros(rows_, cols_);
            e_ = cv::Mat_<FT>::zeros(rows_, cols_);
            zeros = cv::Mat_<FT>::zeros(rows_, cols_);
            return true;
        }

        void init() {
            this->add("grad_numScales", std::bind(&PCSqf::numScales, this, std::placeholders::_1), "Number of scales.");
            this->add("grad_k", std::bind(&PCSqf::k, this, std::placeholders::_1), "k.");
            this->add("grad_cutOff", std::bind(&PCSqf::cutOff, this, std::placeholders::_1), "cutOff.");
            this->add("grad_g", std::bind(&PCSqf::g, this, std::placeholders::_1), "g.");
            this->add("grad_deviationGain", std::bind(&PCSqf::deviationGain, this, std::placeholders::_1), "deviationGain.");
            this->add("grad_noiseMethod", std::bind(&PCSqf::noiseMethod, this, std::placeholders::_1), "Noise method: 0 = none, -1 = dsdf, -2 = sdf, > 0 = threshold.");
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

        PCSqf(FT scale = 1, FT muls = 2, FT kernel_spacing = 1, int nscale = 4, FT kn = 3, FT cutOff = 0.5, FT g = 10, FT deviationGain = static_cast<FT>(1.5), double noiseMethod = -1, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureSF<IT, FT, P>(scale, muls, kernel_spacing, int_lower, int_upper), nscale_(nscale), k_(kn), cutOff_(cutOff), g_(g), dGain_(deviationGain), nMethod_(noiseMethod), eps_(static_cast<FT>(0.0001)) { init(); }

        PCSqf(FT scale, FT l, int k, FT kernel_spacing, int nscale, FT kn, FT cutOff, FT g, FT deviationGain, double noiseMethod, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureSF<IT, FT, P>(scale, l, k, kernel_spacing, int_lower, int_upper), nscale_(nscale), k_(kn), cutOff_(cutOff), g_(g), dGain_(deviationGain), nMethod_(noiseMethod), eps_(static_cast<FT>(0.0001)) { init(); }

        PCSqf(const ValueManager::NameValueVector &options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureSF<IT, FT, P>(static_cast<FT>(1), static_cast<FT>(2), static_cast<FT>(1),int_lower, int_upper), nscale_(4), k_(3), cutOff_(0.5), g_(10), dGain_(1.5), nMethod_(-1), eps_(static_cast<FT>(0.0001)) {
            init();
            value(options);
        }

        PCSqf(ValueManager::InitializerList options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureSF<IT, FT, P>(static_cast<FT>(1), static_cast<FT>(2), static_cast<FT>(1), int_lower, int_upper), nscale_(4), k_(3), cutOff_(0.5), g_(10), dGain_(1.5), nMethod_(-1), eps_(static_cast<FT>(0.0001)) {
            init();
            value(options);
        }

        Value numScales(const Value &ns = Value::NAV()) { if (ns.type()) { nscale_ = ns; this->max_response(); } return nscale_; }
        Value k(const Value &kv = Value::NAV()) { if (kv.type()) k_ = kv; return k_; }
        Value cutOff(const Value &cutOff = Value::NAV()) { if (cutOff.type()) cutOff_ = cutOff; return cutOff_; }
        Value g(const Value &gv = Value::NAV()) { if (gv.type()) g_ = gv; return g_; }
        Value deviationGain(const Value &g = Value::NAV()) { if (g.type()) dGain_ = g; return dGain_; }
        Value noiseMethod(const Value &n = Value::NAV()) { if (n.type()) nMethod_ = n; return nMethod_; }

        virtual void process(const cv::Mat& img) {
            cv::Mat_<FT> e, ox, oy, An, maxAn, tmp;
            cv::Mat_<std::complex<FT>> tmpc;

            if (!updateFilter(img, false)) {
                sumAn.setTo(0);
                e_.setTo(0);
                ox_.setTo(0);
                oy_.setTo(0);
            }

            odd_done_ = false;
            dir_done_ = false;
            even_done_ = true;
            phase_done_ = false;
            energy_done_ = false;
            pc_done_ = false;

            FT tau = 1;

            cv::Mat_<FT> src;
            if (img.type() != cv::DataType<FT>::type)
                img.convertTo(src, cv::DataType<FT>::type);
            else
                src = img;

            if (cols_ < cols_ext_ || rows_ < rows_ext_)
                cv::copyMakeBorder(src, src, 0, rows_ext_ - rows_, 0, cols_ext_ - cols_, cv::BORDER_REPLICATE);


#ifdef USE_PERIODIC_FFT //slower, but removes artifacts
            imgf_ = perfft2<FT>(src);
#else       
            imgf_ = fft2<FT>(src);
#endif

            for (int s = 0; s != nscale_; ++s) {
                e = real(ifft2(multiply(imgf_, fe_[s])));
                tmpc = ifft2(multiply(imgf_, fo_[s]));

                if (cols_ < cols_ext_ || rows_ < rows_ext_) {
                    e.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
                    tmpc.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
                }

                splitT(tmpc, oy, ox);

                cv::sqrt(e.mul(e) + ox.mul(ox) + oy.mul(oy), An);
                sumAn += An;
                e_ += e;
                ox_ += ox;
                oy_ += oy;

                if (s == 0) {
                    if (nMethod_ == -1)
                        tau = median(sumAn) / std::sqrt(std::log(static_cast<FT>(4)));
                    else if (nMethod_ == -2)
                        tau = rayleighMode(sumAn);
                    An.copyTo(maxAn);
                }
                else
                    maxAn = cv::max(maxAn, An);
            }

            cv::divide(sumAn, (maxAn + eps_), tmp);
            tmp -= 1;
            tmp /= (nscale_ - 1);
            cv::exp((cutOff_ - tmp)*g_, weight);
            weight += 1;
            weight = 1 / weight;

            if (nMethod_ >= 0)
                T_ = static_cast<FT>(nMethod_);
            else {
                FT totalTau = tau * (1 - std::pow((1 / muls_), nscale_)) / (1 - (1 / muls_));
                FT EstNoiseEnergyMean = totalTau*std::sqrt(static_cast<FT>(CV_PI) / 2);
                FT EstNoiseEnergySigma = totalTau*std::sqrt((4 - static_cast<FT>(CV_PI)) / 2);

                T_ = EstNoiseEnergyMean + k_*EstNoiseEnergySigma;
            }
        }

        cv::Mat phaseCongruency() const {
            if (!pc_done_) {
                this->energy();
                cv::Mat_<FT> tmp1, tmp2;
                cv::divide(energy_, sumAn + eps_, tmp1);
                cv::max(1 - dGain_ * acos(tmp1), zeros, tmp1);
                cv::max(energy_ - T_, zeros, tmp2);
                cv::divide(tmp2, energy_ + eps_, tmp2);
                pc_ = weight.mul(tmp1.mul(tmp2));
                pc_done_ = true;
            }

            return pc_;
        }

        EnergyRange phaseCongruencyRange() const {
            return EnergyRange(0, 1);
        }

        inline FT phaseCongruencyThreshold(double val) const {
            return static_cast<FT>(val);
        }

        //! get name of gradient operator
        virtual std::string name() const {
            return "pc_sqf";
        }

        //! generic interface to get processed data
        virtual FilterResults results() const {
            FilterResults ret;
            ret["even"] = FilterData(this->even(), this->evenRange());
            ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
            ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
            ret["odd"] = FilterData(this->odd(), this->oddRange());
            ret["dir"] = FilterData(this->direction(), this->directionRange());
            ret["energy"] = FilterData(this->energy(), this->energyRange());
            ret["phase"] = FilterData(this->phase(), this->phaseRange());
            ret["pc"] = FilterData(phaseCongruency(), this->phaseCongruencyRange());
            return ret;
        }

        using QuadratureSF<IT, FT, P>::values;
        using QuadratureSF<IT, FT, P>::valuePair;
        using QuadratureSF<IT, FT, P>::value;
        using QuadratureSF<IT, FT, P>::intensityRange;
        using QuadratureSF<IT, FT, P>::even;
        using QuadratureSF<IT, FT, P>::evenRange;
        using QuadratureSF<IT, FT, P>::evenThreshold;
        using QuadratureSF<IT, FT, P>::odd;
        using QuadratureSF<IT, FT, P>::oddRange;
        using QuadratureSF<IT, FT, P>::oddThreshold;
        using QuadratureSF<IT, FT, P>::oddx;
        using QuadratureSF<IT, FT, P>::oddy;
        using QuadratureSF<IT, FT, P>::oddGradRange;
        using QuadratureSF<IT, FT, P>::direction;
        using QuadratureSF<IT, FT, P>::directionRange;
        using QuadratureSF<IT, FT, P>::phase;
        using QuadratureSF<IT, FT, P>::phaseRange;
        using QuadratureSF<IT, FT, P>::normType;
    };

    template<class IT = uchar, class FT = float, template<typename, typename> class P = Polar>
    class PCLSqf : public PhaseCongruencyLaplaceI<FT>, public QuadratureSF<IT, FT, P> {
    public:
        using QuadratureSF<IT, FT, P>::PI2;

        static inline std::complex<FT> docpfds(FT u, FT v, FT s, FT m) {
            FT a = -PI2*std::sqrt(u*u + v*v)*s, b = m*std::exp(a*m) - std::exp(a);
            return std::complex<FT>(PI2 * -u * b, PI2 * v * b);
        }
        static inline FT dopfds(FT u, FT v, FT s, FT m) {
            FT a = PI2*std::sqrt(u*u + v*v);
            return a * (m*std::exp(-a*s*m) - std::exp(-a*s));
        }
    private:

        using QuadratureSF<IT, FT, P>::fe_;
        using QuadratureSF<IT, FT, P>::fo_;
        using QuadratureSF<IT, FT, P>::imgf_;

        using QuadratureSF<IT, FT, P>::ox_;
        using QuadratureSF<IT, FT, P>::oy_;
        using QuadratureSF<IT, FT, P>::e_;
        using QuadratureSF<IT, FT, P>::rows_;
        using QuadratureSF<IT, FT, P>::cols_;
        using QuadratureSF<IT, FT, P>::rows_ext_;
        using QuadratureSF<IT, FT, P>::cols_ext_;

        using QuadratureSF<IT, FT, P>::kspacing_;
        using QuadratureSF<IT, FT, P>::scale_;
        using QuadratureSF<IT, FT, P>::muls_;
        using QuadratureSF<IT, FT, P>::evenRange_;
        using QuadratureSF<IT, FT, P>::oddRange_;


        cv::Mat_<FT> dfe_;
        cv::Mat_<std::complex<FT>> dfo_;

        mutable bool plaplace_done_;
        mutable cv::Mat_<FT> oxds_, oyds_, eds_, lx_, ly_;

        virtual bool updateFilter(const cv::Mat img, bool force = true) {
            // if force is of, only renew if image size has changed
            if (!force && img.rows == rows_ && img.cols == cols_)
                return false;

            rows_ = img.rows;
            cols_ = img.cols;
            rows_ext_ = cv::getOptimalDFTSize(img.rows);
            cols_ext_ = cv::getOptimalDFTSize(img.cols);

            fe_ = ifftshift(QuadratureSF<IT, FT, P>::createFilter(rows_ext_, cols_ext_, kspacing_, scale_, muls_, QuadratureSF<IT, FT, P>::dopf));
            fo_ = ifftshift(QuadratureSF<IT, FT, P>::createFilterC(rows_ext_, cols_ext_, kspacing_, scale_, muls_, QuadratureSF<IT, FT, P>::docpf));
            dfe_ = ifftshift(QuadratureSF<IT, FT, P>::createFilter(rows_ext_, cols_ext_, kspacing_, scale_, muls_, dopfds));
            dfo_ = ifftshift(QuadratureSF<IT, FT, P>::createFilterC(rows_ext_, cols_ext_, kspacing_, scale_, muls_, docpfds));
            return true;
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

        PCLSqf(FT scale = 1, FT muls = 2, FT kernel_spacing = 1, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureSF<IT, FT, P>(scale, muls, kernel_spacing, int_lower, int_upper) { }

        PCLSqf(FT scale, FT l, int k, FT kernel_spacing, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureSF<IT, FT, P>(scale, l, k, kernel_spacing, int_lower, int_upper) {}

        PCLSqf(const ValueManager::NameValueVector &options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureSF<IT, FT, P>(options, int_lower, int_upper) {}

        PCLSqf(ValueManager::InitializerList options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureSF<IT, FT, P>(options, int_lower, int_upper) {}

        virtual void process(const cv::Mat& img) {
            plaplace_done_ = false;
            QuadratureSF<IT, FT, P>::process(img);
        }

        void pcLaplace(cv::Mat &pdsx, cv::Mat &pdsy) const {
            if (!plaplace_done_) {
                even(); odd();
                eds_ = real(ifft2(multiply(imgf_, dfe_)));
                cv::Mat_<std::complex<FT>> tmp = ifft2(multiply(imgf_, dfo_));

                if (cols_ < cols_ext_ || rows_ < rows_ext_) {
                    tmp.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
                    eds_.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
                }

                splitT(tmp, oyds_, oxds_);
                lx_ = e_.mul(oxds_) - ox_.mul(eds_);
                ly_ = e_.mul(oyds_) - oy_.mul(eds_);
                plaplace_done_ = true;
            }
            /*showMat("fe", fftshift(fe_));
            showMat("fox", fftshift(imag(fo_)));
            showMat("foy", fftshift(real(fo_)));
            showMat("dfe", fftshift(dfe_));
            showMat("dfox", fftshift(imag(dfo_)));
            showMat("dfoy", fftshift(real(dfo_)));
            showMat("gxf", abs(ox_));
            showMat("gyf", abs(oy_));*/
            //showMat("ef", e_);
            //showMat("magf", magnitude());
            //showMat("energyf", energy());
            /*showMat("edsf", abs(eds_));
            showMat("gxdsf", abs(oxds_));
            showMat("gydsf", abs(oyds_));
            showMat("lxf", abs(lx_));
            showMat("lyf", abs(ly_));
            cv::Mat tmp;
            P<FT, FT>::magnitude(lx_, ly_, tmp);
            showMat("lf", tmp);
            cv::waitKey();*/
            pdsx = lx_; pdsy = ly_;
        }

        inline cv::Mat pclMag() const {
            cv::Mat tmp;
            P<FT, FT>::magnitude(this->pclx(), this->pcly(), tmp);
            return tmp;
        }

        EnergyRange pcLaplaceRange() const {
            FT val = evenRange_.upper*evenRange_.upper + oddRange_.upper * oddRange_.upper;
            return EnergyRange(-val, val);
        }

        inline FT pcLaplaceThreshold(double val) const {
            return static_cast<FT>(pcLaplaceRange().size() * val * val);
        }

        /*cv::Mat laplace() const {
            return this->pclx()+this->pcly();
        }

        EnergyRange laplaceRange() const {
            return pcLaplaceRange();
        }

        FT laplaceThreshold(double val) const {
            return pcLaplaceThreshold(val);
        }*/

        //! generic interface to get processed data
        virtual FilterResults results() const {
            FilterResults ret;
            ret["even"] = FilterData(this->even(), this->evenRange());
            ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
            ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
            ret["odd"] = FilterData(this->odd(), this->oddRange());
            ret["dir"] = FilterData(this->direction(), this->directionRange());
            ret["energy"] = FilterData(this->energy(), this->energyRange());
            ret["phase"] = FilterData(this->phase(), this->phaseRange());
            ret["pclx"] = FilterData(this->pclx(), this->pcLaplaceRange());
            ret["pcly"] = FilterData(this->pcly(), this->pcLaplaceRange());
            ret["pclmag"] = FilterData(this->pclMag(), this->pcLaplaceRange());
            return ret;
        }

        //! get name of gradient operator
        virtual std::string name() const {
            return "pcl_sqf";
        }

        virtual IntensityRange intensityRange() const {
            return QuadratureSF<IT, FT, P>::intensityRange();
        }

        using QuadratureSF<IT, FT, P>::values;
        using QuadratureSF<IT, FT, P>::valuePair;
        using QuadratureSF<IT, FT, P>::value;
        using QuadratureSF<IT, FT, P>::intensityRange;
        using QuadratureSF<IT, FT, P>::even;
        using QuadratureSF<IT, FT, P>::evenRange;
        using QuadratureSF<IT, FT, P>::evenThreshold;
        using QuadratureSF<IT, FT, P>::odd;
        using QuadratureSF<IT, FT, P>::oddRange;
        using QuadratureSF<IT, FT, P>::oddThreshold;
        using QuadratureSF<IT, FT, P>::oddx;
        using QuadratureSF<IT, FT, P>::oddy;
        using QuadratureSF<IT, FT, P>::oddGradRange;
        using QuadratureSF<IT, FT, P>::direction;
        using QuadratureSF<IT, FT, P>::directionRange;
        using QuadratureSF<IT, FT, P>::phase;
        using QuadratureSF<IT, FT, P>::phaseRange;
        using QuadratureSF<IT, FT, P>::normType;
    };

    template<class IT = uchar, class GT = float, class FT = float, template<typename, typename> class P = Polar>
    class PCLSq : public PhaseCongruencyLaplaceI<FT>, public QuadratureS<IT, GT, FT, P> {
    public:
        using QuadratureS<IT, GT, FT, P>::createFilter;
        static inline FT docpds(FT x, FT y, FT s, FT m) {
            FT a = (s*s + x*x + y*y), b = (m*m * s*s + x*x + y*y);
            return (3 * m*m*s*x) / (2 * static_cast<FT>(CV_PI) * std::sqrt(b*b*b*b*b)) -
                (3 * s*x) / (2 * static_cast<FT>(CV_PI) * std::sqrt(a*a*a*a*a));
        }
        static inline FT dopds(FT x, FT y, FT s, FT m) {
            FT a = (s*s + x*x + y*y), b = (m*m * s*s + x*x + y*y);
            return 1 / (2 * static_cast<FT>(CV_PI) * std::sqrt(a*a*a)) -
                (3 * s*s) / (2 * static_cast<FT>(CV_PI) * std::sqrt(a*a*a*a*a)) -
                m / (2 * static_cast<FT>(CV_PI) * std::sqrt(b*b*b)) +
                (3 * m*m*m*s*s) / (2 * static_cast<FT>(CV_PI) * std::sqrt(b*b*b*b*b));
        }
    private:

        cv::Mat_<FT> koxds_, koyds_, keds_;

        mutable bool plaplace_done_;
        mutable cv::Mat_<FT> oxds_, oyds_, eds_, lx_, ly_;

        using QuadratureS<IT, GT, FT, P>::img_;

        using QuadratureS<IT, GT, FT, P>::ox_;
        using QuadratureS<IT, GT, FT, P>::oy_;
        using QuadratureS<IT, GT, FT, P>::e_;
        

        using QuadratureS<IT, GT, FT, P>::kspacing_;
        using QuadratureS<IT, GT, FT, P>::scale_;
        using QuadratureS<IT, GT, FT, P>::kscale_;
        using QuadratureS<IT, GT, FT, P>::ksize_;
        using QuadratureS<IT, GT, FT, P>::muls_;

        using QuadratureS<IT, GT, FT, P>::evenRange_;
        using QuadratureS<IT, GT, FT, P>::anchor;

        virtual void create_kernels() {
            QuadratureS<IT, GT, FT, P>::create_kernels();
            koxds_ = createFilter(ksize_, kspacing_, scale_, muls_, docpds) * kscale_;
            keds_ = createFilter(ksize_, kspacing_, scale_, muls_, dopds) * kscale_;

            if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
                koxds_.convertTo(koxds_, cv::DataType<GT>::type);
                keds_.convertTo(keds_, cv::DataType<GT>::type);
            }

            // zero dc
#ifndef DISABLE_DC_ZERO_FIX
            keds_ -= cv::sum(keds_)[0] / (ksize_*ksize_);
#endif

            koyds_ = koxds_.t();
        }


    public:
        typedef IT img_type;
        typedef GT grad_type;
        typedef FT mag_type;
        typedef FT energy_type;
        typedef FT dir_type;
        typedef FT phase_type;

        typedef Range<IT> IntensityRange;
        typedef Range<GT> GradientRange;
        typedef Range<FT> MagnitudeRange;
        typedef Range<FT> EnergyRange;
        typedef Range<FT> DirectionRange;
        typedef Range<FT> PhaseRange;

        PCLSq(FT scale = 1, FT muls = 2, int kernel_size = 5, FT kernel_spacing = 1, FT kernel_scale = 1, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureS<IT, GT, FT, P>(scale, muls, kernel_size, kernel_spacing, kernel_scale, int_lower, int_upper) { create_kernels(); }

        PCLSq(FT scale, FT l, int k, int kernel_size = 5, FT kernel_spacing = 1, FT kernel_scale = 1, IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : QuadratureS<IT, GT, FT, P>(scale, l, k, kernel_size, kernel_spacing, kernel_scale, int_lower, int_upper) { create_kernels(); }

        PCLSq(const ValueManager::NameValueVector &options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureS<IT, GT, FT, P>(options, int_lower, int_upper) { create_kernels(); }

        PCLSq(ValueManager::InitializerList options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : QuadratureS<IT, GT, FT, P>(options, int_lower, int_upper) { create_kernels(); }

        virtual void process(const cv::Mat& img) {
            plaplace_done_ = false;
            QuadratureS<IT, GT, FT, P>::process(img);
        }

        void pcLaplace(cv::Mat &pdsx, cv::Mat &pdsy) const {
            if (!plaplace_done_) {
                even();
                cv::filter2D(img_, oxds_, cv::DataType<GT>::type, koxds_, anchor, 0, cv::BORDER_REFLECT_101);
                cv::filter2D(img_, oyds_, cv::DataType<GT>::type, koyds_, anchor, 0, cv::BORDER_REFLECT_101);
                cv::filter2D(img_, eds_, cv::DataType<GT>::type, keds_, anchor, 0, cv::BORDER_REFLECT_101);
                lx_ = e_.mul(oxds_) - ox_.mul(eds_);
                ly_ = e_.mul(oyds_) - oy_.mul(eds_);
                plaplace_done_ = true;
            }

            /*showMat("gx", abs(gx_));
            showMat("gy", abs(gy_));*/
            //showMat("e", e_);
            //showMat("mag", magnitude());
            //showMat("energy", energy());
            /*showMat("eds", abs(eds_));
            showMat("gxds", abs(oxds_));
            showMat("gyds", abs(oyds_));
            showMat("lx", abs(lx_));
            showMat("ly", abs(ly_));
            cv::Mat tmp;
            P<FT, FT>::magnitude(lx_, ly_, tmp);
            showMat("l", tmp);
            cv::waitKey();*/
            pdsx = lx_; pdsy = ly_;
        }

        inline cv::Mat pclMag() const {
            
            cv::Mat tmp;
            P<FT, FT>::magnitude(this->pclx(), this->pcly(), tmp);
            return tmp;
        }

        EnergyRange pcLaplaceRange() const {
            FT val = evenRange_.upper*evenRange_.upper + oddRange().upper * oddRange().upper;
            return EnergyRange(-val, val);
        }

        inline FT pcLaplaceThreshold(double val) const {
            return static_cast<FT>(pcLaplaceRange().size() * val * val);
        }

        /*cv::Mat laplace() const {
            return this->pclx()+this->pcly();
        }

        GradientRange laplaceRange() const {
            return pcLaplaceRange();
        }

        GT laplaceThreshold(double val) const {
            return static_cast<GT>(pcLaplaceThreshold(val));
        }*/

        //! generic interface to get processed data
        virtual FilterResults results() const {
            FilterResults ret;
            ret["even"] = FilterData(this->even(), this->evenRange());
            ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
            ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
            ret["odd"] = FilterData(this->odd(), this->oddRange());
            ret["dir"] = FilterData(this->direction(), this->directionRange());
            ret["energy"] = FilterData(this->energy(), this->energyRange());
            ret["phase"] = FilterData(this->phase(), this->phaseRange());
            ret["pclx"] = FilterData(this->pclx(), this->pcLaplaceRange());
            ret["pcly"] = FilterData(this->pcly(), this->pcLaplaceRange());
            ret["pclmag"] = FilterData(this->pclMag(), this->pcLaplaceRange());
            return ret;
        }

        //! get name of gradient operator
        virtual std::string name() const {
            return "pcl_sq";
        }

        virtual IntensityRange intensityRange() const {
            return QuadratureS<IT, GT, FT, P>::intensityRange();
        }

        using QuadratureS<IT, GT, FT, P>::values;
        using QuadratureS<IT, GT, FT, P>::valuePair;
        using QuadratureS<IT, GT, FT, P>::value;
        using QuadratureS<IT, GT, FT, P>::intensityRange;
        using QuadratureS<IT, GT, FT, P>::even;
        using QuadratureS<IT, GT, FT, P>::evenRange;
        using QuadratureS<IT, GT, FT, P>::evenThreshold;
        using QuadratureS<IT, GT, FT, P>::odd;
        using QuadratureS<IT, GT, FT, P>::oddRange;
        using QuadratureS<IT, GT, FT, P>::oddThreshold;
        using QuadratureS<IT, GT, FT, P>::oddx;
        using QuadratureS<IT, GT, FT, P>::oddy;
        using QuadratureS<IT, GT, FT, P>::oddGradRange;
        using QuadratureS<IT, GT, FT, P>::direction;
        using QuadratureS<IT, GT, FT, P>::directionRange;
        using QuadratureS<IT, GT, FT, P>::phase;
        using QuadratureS<IT, GT, FT, P>::phaseRange;
        using QuadratureS<IT, GT, FT, P>::normType;
    };

}

#endif
#endif
