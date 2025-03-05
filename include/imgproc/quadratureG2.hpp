
#ifndef _QUADRATUREG2_HPP_
#define _QUADRATUREG2_HPP_
#ifdef __cplusplus

#include <opencv2/core/core.hpp>
#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>


//#define ENABLE_G2_FULL_STEER

namespace lsfm {

    template<class IT = uchar, class FT = float, template<typename, typename> class P = Polar>
    class QuadratureG2 : public Quadrature<IT, FT, FT, FT, FT> {

    public:
        static FT G21(FT x) { return static_cast<FT>(0.9213 * (2.0*x*x - 1.0) * exp(-x*x)); }
        static FT G22(FT x) { return exp(-x*x); }
        static FT G23(FT x) { return sqrt(static_cast<FT>(1.8430)) * x * exp(-x*x); }

        static FT H21(FT x) { return static_cast<FT>(0.9780 * (-2.254 * x + x*x*x)) * exp(-x*x); }
        static FT H22(FT x) { return exp(-x*x); }
        static FT H23(FT x) { return x * exp(-x*x); }
        static FT H24(FT x) { return static_cast<FT>(0.9780 * (-0.7515 + x*x)) * exp(-x*x); }

        typedef FT(*KernelType)(FT x);

        static cv::Mat_<FT> createFilter(int width, FT spacing, KernelType f) {
            width = width / 2;
            cv::Mat_<FT> kernel(1, width*2+1);
            for (int i = -width; i <= width; ++i)
                kernel(i + width) = f(i*spacing);

            return kernel;
        }
    
    private:

        cv::Point anchor;

        // the kernels
        cv::Mat_<FT> m_dx, m_dy;
        cv::Mat_<FT> m_g1, m_g2, m_g3, m_h1, m_h2, m_h3, m_h4;
        cv::Mat_<FT> m_g2a, m_g2b, m_g2c, m_h2a, m_h2b, m_h2c, m_h2d;
        cv::Mat_<FT> m_c1, m_c2, m_c3, m_theta;

#ifdef ENABLE_G2_FULL_STEER
        cv::Mat_<FT> m_s;
#endif

        mutable cv::Mat energy_, phase_, ox_, oy_, e_, o_;

        Range<FT> oddRange_, evenRange_, energyRange_;
        using Quadrature<IT, FT, FT, FT, FT>::intRange_;

        int ksize_;
        FT kspacing_;

        mutable bool energy_done_, phase_done_, oxy_done_, eo_done_;

        void create_kernels() {
            // Create separable filters for G2
            m_g1 = createFilter(ksize_, kspacing_, G21);
            m_g2 = createFilter(ksize_, kspacing_, G22);
            m_g3 = createFilter(ksize_, kspacing_, G23);

            // Create separable filters for H2
            m_h1 = createFilter(ksize_, kspacing_, H21);
            m_h2 = createFilter(ksize_, kspacing_, H22);
            m_h3 = createFilter(ksize_, kspacing_, H23);
            m_h4 = createFilter(ksize_, kspacing_, H24);

            // zero dc
#ifndef DISABLE_DC_ZERO_FIX
            m_g1 -= cv::sum(m_g1)[0] / ksize_;
            m_h4 -= cv::sum(m_h4)[0] / ksize_;
#endif
            max_response();
        }

        void max_response() {

            cv::Mat_<IT> tmp(2* ksize_,2* ksize_);
            tmp.setTo(this->intRange_.lower);
            tmp.colRange(0,ksize_).setTo(this->intRange_.upper);
            process(tmp);
            double vmin, vmax;
            cv::minMaxIdx(energy(), &vmin, &vmax);
            energyRange_.upper = static_cast<FT>(vmax);
            cv::minMaxIdx(odd(), &vmin, &vmax);
            oddRange_.upper = static_cast<FT>(vmax);
            cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
            evenRange_.upper = static_cast<FT>(vmax);;
            evenRange_.lower = -evenRange_.upper;
        }

        void init() {
            this->add("grad_kernel_size", std::bind(&QuadratureG2<IT, FT, P>::valueKernelSize, this, std::placeholders::_1), "Kernel size for Quadrature-Operators.");
            this->add("grad_kernel_spacing", std::bind(&QuadratureG2<IT, FT, P>::valueKernelSpacing, this, std::placeholders::_1), "Spacing for a single step for Quadrature-Operators.");

            create_kernels();
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


        QuadratureG2(int kernel_size = 9, FT kernel_spacing = static_cast<FT>(0.782), IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
            : Quadrature<IT, FT, FT, FT, FT>(int_lower,int_upper), ksize_(kernel_size), kspacing_(kernel_spacing), anchor(-1, -1) {
            init();
        }

        QuadratureG2(const ValueManager::NameValueVector &options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper), ksize_(9), kspacing_(static_cast<FT>(0.782)), anchor(-1, -1) {
            init();
            this->value(options);
        }

        QuadratureG2(ValueManager::InitializerList options, img_type int_lower = std::numeric_limits<img_type>::lowest(), img_type int_upper = std::numeric_limits<img_type>::max())
            : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper), ksize_(9), kspacing_(static_cast<FT>(0.782)), anchor(-1, -1) {
            init();
            this->value(options);
        }

        Value valueKernelSize(const Value &ks = Value::NAV()) { if (ks.type()) kernelSize(ks); return ksize_; }

        //! get kernel size
        int kernelSize() const { return ksize_; }

        //! set kernel size (range 3-99, has to be odd, even will be corrected to ksize+1)
        //! Note: large kernels needs larger GT type like int or long long int
        void kernelSize(int ks) {
            if (ks == ksize_)
                return;

            if (ks < 3)
                ks = 3;
            if (ks % 2 == 0)
                ++ks;
            if (ks > 99)
                ks = 99;
            ksize_ = ks;
            create_kernels();
        }

        Value valueKernelSpacing(const Value &ks = Value::NAV()) { if (ks.type()) kernelSpacing(ks); return kspacing_; }

        //! get kernel spacing
        FT kernelSpacing() const { return kspacing_; }

        //! set kernel size (range 3-99, has to be odd, even will be corrected to ksize+1)
        //! Note: large kernels needs larger GT type like int or long long int
        void kernelSpacing(FT ks) {
            if (ks == kspacing_ || ks <= 0)
                return;
            kspacing_ = ks;
            create_kernels();
        }

        void process(const cv::Mat& img) {

            energy_done_ = false;
            oxy_done_ = false;
            eo_done_ = false;
            phase_done_ = false;

            // compute the outputs
            cv::sepFilter2D(img, m_g2a, cv::DataType<FT>::type, m_g1, m_g2, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_g2b, cv::DataType<FT>::type, m_g3, m_g3, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_g2c, cv::DataType<FT>::type, m_g2, m_g1, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_h2a, cv::DataType<FT>::type, m_h1, m_h2, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_h2b, cv::DataType<FT>::type, m_h4, m_h3, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_h2c, cv::DataType<FT>::type, m_h3, m_h4, anchor, 0, cv::BORDER_REFLECT_101);
            cv::sepFilter2D(img, m_h2d, cv::DataType<FT>::type, m_h2, m_h1, anchor, 0, cv::BORDER_REFLECT_101);

            cv::Mat g2aa = m_g2a.mul(m_g2a); // g2a*
            cv::Mat g2ab = m_g2a.mul(m_g2b);
#ifdef ENABLE_G2_FULL_STEER
            cv::Mat g2ac = m_g2a.mul(m_g2c);
            cv::Mat g2bb = m_g2b.mul(m_g2b); // g2b*
#endif
            cv::Mat g2bc = m_g2b.mul(m_g2c);
            cv::Mat g2cc = m_g2c.mul(m_g2c); // g2c*
            cv::Mat h2aa = m_h2a.mul(m_h2a); // h2a*
            cv::Mat h2ab = m_h2a.mul(m_h2b);
            cv::Mat h2ac = m_h2a.mul(m_h2c);
            cv::Mat h2ad = m_h2a.mul(m_h2d);
            cv::Mat h2bb = m_h2b.mul(m_h2b); // h2b*
            cv::Mat h2bc = m_h2b.mul(m_h2c);
            cv::Mat h2bd = m_h2b.mul(m_h2d);
            cv::Mat h2cc = m_h2c.mul(m_h2c); // h2c*
            cv::Mat h2cd = m_h2c.mul(m_h2d);
            cv::Mat h2dd = m_h2d.mul(m_h2d); // h2d*

#ifdef ENABLE_G2_FULL_STEER
            m_c1 = 0.5*(g2bb)+0.25*(g2ac)+0.375*(g2aa + g2cc) + 0.3125*(h2aa + h2dd) + 0.5625*(h2bb + h2cc) + 0.375*(h2ac + h2bd);
#endif
            m_c2 = 0.5*(g2aa - g2cc) + 0.46875*(h2aa - h2dd) + 0.28125*(h2bb - h2cc) + 0.1875*(h2ac - h2bd);
            m_c3 = (-g2ab) - g2bc - (0.9375*(h2cd + h2ab)) - (1.6875*(h2bc)) - (0.1875*(h2ad));

#ifdef ENABLE_G2_FULL_STEER
            P<FT,FT>::cart2Polar(m_c2, m_c3, m_s, m_theta);
#else
            P<FT, FT>::phase(m_c2, m_c3, m_theta);
#endif
            m_theta *= -0.5;
        }

        cv::Mat kernel() const {
            return m_g1 * m_g2.t();
        }

        //! get direction
        cv::Mat direction() const {
            return m_theta;
        }

        DirectionRange directionRange() const {
            return P<FT,FT>::range();
        }

#ifdef ENABLE_G2_FULL_STEER
        //! get direction strength
        cv::Mat strength() const {
            return m_s;
        }
#endif

        //! get odd response
        void odd(cv::Mat& ox, cv::Mat& oy) const {
            if (!oxy_done_) {
                P<FT,FT>::polar2Cart(m_theta, ox_, oy_);
                oxy_done_ = true;
            }
            ox = ox_;
            oy = oy_;
        }

        //! steer to theta to get maximal responses
        void steer(const cv::Mat &theta, cv::Mat &g2, cv::Mat &h2) const
        {
            // Create the steering coefficients, then compute G2 and H2 at orientation theta:
            cv::Mat ct, ct2, ct3, st, st2, st3;
            P<FT,FT>::polarToCart(-theta, ct, st);
            ct2 = ct.mul(ct), ct3 = ct2.mul(ct), st2 = st.mul(st), st3 = st2.mul(st);
            g2 = ct2.mul(m_g2a) + (-2.0 * ct.mul(st).mul(m_g2b)) + (st2.mul(m_g2c));
            h2 = ct3.mul(m_h2a) + (-3.0 * ct2.mul(st).mul(m_h2b)) + (3.0 * ct.mul(st2).mul(m_h2c)) + (-st3.mul(m_h2d));
        }

        //! steer to theta to get maximal responses
        void steer(const cv::Mat &gx, const cv::Mat &gy, cv::Mat &g2, cv::Mat &h2) const
        {
            // Create the steering coefficients, then compute G2 and H2 at orientation theta:
            cv::Mat ct2, ct3, st2, st3, mgy = -gy;
            ct2 = gx.mul(gx), ct3 = ct2.mul(gx), st2 = mgy.mul(mgy), st3 = st2.mul(mgy);
            g2 = ct2.mul(m_g2a) + (-2.0 * gx.mul(mgy).mul(m_g2b)) + (st2.mul(m_g2c));
            h2 = ct3.mul(m_h2a) + (-3.0 * ct2.mul(mgy).mul(m_h2b)) + (3.0 * gx.mul(st2).mul(m_h2c)) + (-st3.mul(m_h2d));
        }

        //! test if even responses are computed
        inline bool isEvenDone() const {
            return eo_done_;
        }

        //! get response of even filter at strongest direction
        cv::Mat even() const {
            if (!eo_done_) {
                cv::Mat t1, t2;
                evenOdd(t1, t2);
            }
            return e_;
        }

        GradientRange evenRange() const {
            return evenRange_;
        }

        //! test if odd responses are computed
        inline bool isOddDone() const {
            return eo_done_;
        }

        //! get response of odd filter at strongest direction
        cv::Mat odd() const {
            if (!eo_done_) {
                cv::Mat t1, t2;
                evenOdd(t1, t2);
            }
            return o_;
        }

        //! get response of even and odd filter at strongest direction
        void evenOdd(cv::Mat &e, cv::Mat &o) const {
            if (!eo_done_) {
                steer(this->oddx(), oy_, e_, o_);
                o_ = abs(o_);
                eo_done_ = true;
            }
            e = e_;
            o = o_;
        }
        
        MagnitudeRange oddRange() const {
            return oddRange_;
        }

        
        //! test if magnitude is computed
        inline bool isEnergyDone() const {
            return energy_done_;
        }

        //! get magnitude
        cv::Mat energy() const {
            if (!energy_done_) {
                P<FT,FT>::magnitude(odd(), e_, energy_);
                energy_done_ = true;
            }
            return energy_;
        }

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
                P<FT,FT>::phase(odd(), e_, phase_);
                phase_done_ = true;
            }
            return phase_;
        }

        //! get direction range ([-PI,PI], [0,2PI] or [0,360])
        PhaseRange phaseRange() const {
            return P<FT,FT>::range();
        }

        //! get phase and energy
        void energyPhase(cv::Mat& en, cv::Mat& ph) const {
            if (!phase_done_ && !energy_done_) {
                P<FT,FT>::cart2Polar(odd(), e_, energy_, phase_);
                phase_done_ = true;
                energy_done_ = true;
            } 
            else if (!phase_done_) {
                phase();
            }
            else if (!energy_done_) {
                energy();
            }
            en = energy_;
            ph = phase_;
        }


        //! get name of gradient operator
        inline std::string name() const {
            return "quadratureG2";
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

#ifdef ENABLE_G2_FULL_STEER
        void computeEnergyAndPhase(const cv::Mat &g2, const cv::Mat &h2, cv::Mat &energy, cv::Mat &phase) const
        {
            P<FT,FT>::cart2Polar(g2, h2, energy, phase);
            //cv::cartToPolar(g2, h2, energy, phase); // [0..2*piI]
            //wrap(phase, phase); // [-pi/2 pi/2]
            //cv::patchNaNs(phase);
            //phase.setTo(10000, energy < 1e-2f);
        }

        // Steer filters at a single pixel:
        void steer(const cv::Point &p, FT theta, FT &g2, FT &h2) const
        {
            // Create the steering coefficients, then compute G2 and H2 at orientation theta:
            FT ct(std::cos(theta)), ct2(ct*ct), ct3(ct2*ct), st(std::sin(theta)), st2(st*st), st3(st2*st);
            FT ga(ct2), gb(-2.0 * ct * st), gc(st2);
            FT ha(ct3), hb(-3.0 * ct2 * st), hc(3.0 * ct * st2), hd(-st3);
            g2 = ga * m_g2a(p) + gb * m_g2b(p) + gc * m_g2c(p);
            h2 = ha * m_h2a(p) + hb * m_h2b(p) + hc * m_h2c(p) + hd * m_h2d(p);
        }

        void steer(const cv::Point &p, FT theta, FT &g2, FT &h2, FT &e, FT &energy, FT &phase) const
        {
            steer(p, theta, g2, h2);
            phase = std::atan2(h2, g2);
            energy = std::sqrt(h2*h2 + g2*g2);

            // Compute oriented energy as a function of angle theta
            FT c2t(std::cos(theta*2.0)), s2t(std::sin(theta*2.0));
            e = m_c1(p) + (c2t * m_c2(p)) + (s2t * m_c3(p));
        }

        // Steer filters across the entire image:
        void steer(FT theta, cv::Mat &g2, cv::Mat &h2) const
        {
            // Create the steering coefficients, then compute G2 and H2 at orientation theta:
            FT ct(std::cos(theta)), ct2(ct*ct), ct3(ct2*ct), st(std::sin(theta)), st2(st*st), st3(st2*st);
            FT ga(ct2), gb(-2.0 * ct * st), gc(st2);
            FT ha(ct3), hb(-3.0 * ct2 * st), hc(3.0 * ct * st2), hd(-st3);
            g2 = ga * m_g2a + gb * m_g2b + gc * m_g2c;
            h2 = ha * m_h2a + hb * m_h2b + hc * m_h2c + hd * m_h2d;
        }

        
        void steer(FT theta, cv::Mat &g2, cv::Mat &h2, cv::Mat &e, cv::Mat &energy, cv::Mat &phase) const
        {
            steer(theta, g2, h2);
            computeEnergyAndPhase(g2, h2, energy, phase);

            // Compute oriented energy as a function of angle theta
            FT c2t(std::cos(theta*2.0)), s2t(std::sin(theta*2.0));
            e = m_c1 + (c2t * m_c2) + (s2t * m_c3);
        }

        void steer(const cv::Mat &theta, cv::Mat &g2, cv::Mat &h2, cv::Mat &e, cv::Mat &energy, cv::Mat &phase) const
        {
            // Create the steering coefficients, then compute G2 and H2 at orientation theta:
            steer(theta, g2, h2);
            computeEnergyAndPhase(g2, h2, energy, phase);

            // Compute oriented energy as a function of angle theta
            cv::Mat c2t, s2t;
            P<FT,FT>::polar2Cart(theta * 2.0, c2t, s2t);
            e = m_c1 + m_c2.mul(c2t) + m_c3.mul(s2t);
        }
#endif
    };

}

#endif
#endif
