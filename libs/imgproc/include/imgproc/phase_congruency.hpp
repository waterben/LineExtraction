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

#ifndef _PHASE_CONGRUENCY_HPP_
#define _PHASE_CONGRUENCY_HPP_
#ifdef __cplusplus

#include <imgproc/quadrature.hpp>

namespace lsfm {

    //! Phase Congruency base class (quadrature extension)
    //! Use ET to define energy type (int, float or double)
    template<class ET>
    class PhaseCongruencyI {
        PhaseCongruencyI(const PhaseCongruencyI&);

    protected:
        PhaseCongruencyI() {}

    public:
        typedef ET energy_type;
        typedef Range<ET> EnergyRange;

        virtual ~PhaseCongruencyI() {}

        //! get phase congruency
        virtual cv::Mat phaseCongruency() const = 0;

        //! get phase congruency range
        virtual EnergyRange phaseCongruencyRange() const {
            return EnergyRange(0, 1);
        }

    };

    //! phase congruency base class helper
    template<class IT, class GT, class MT, class ET, class DT>
    class PhaseCongruency : public PhaseCongruencyI<ET>, public QuadratureI<IT, GT, MT, ET, DT>{
        PhaseCongruency();
        PhaseCongruency(const PhaseCongruency&);
    protected:
        Range<IT> intRange_;

        PhaseCongruency(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
            if (intRange_.lower > intRange_.upper)
                intRange_.swap();
        }

    public:
        typedef IT img_type;
        typedef GT grad_type;
        typedef MT mag_type;
        typedef ET energy_type;
        typedef DT dir_type;
        typedef DT phase_type;

        typedef Range<IT> IntensityRange;
        typedef Range<GT> GradientRange;
        typedef Range<MT> MagnitudeRange;
        typedef Range<ET> EnergyRange;
        typedef Range<DT> DirectionRange;
        typedef Range<DT> PhaseRange;


        //! get image intensity range (for single channel)
        IntensityRange intensityRange() const {
            return intRange_;
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
            ret["pc"] = FilterData(this->phaseCongruency(), this->phaseCongruencyRange());
            return ret;
        }
    };


    //! Phase Congruency laplace base class (local energy)
    //! Use ET to define energy type (int, float or double)
    template<class ET>
    class PhaseCongruencyLaplaceI  {
        PhaseCongruencyLaplaceI(const PhaseCongruencyLaplaceI&);

    protected:
        PhaseCongruencyLaplaceI() {}

    public:
        typedef ET energy_type;

        typedef Range<ET> EnergyRange;

        virtual ~PhaseCongruencyLaplaceI() {}

        //! get single pc laplace responses
        virtual void pcLaplace(cv::Mat &lx, cv::Mat &ly) const = 0;

        //! get x response of pc laplace filter
        virtual cv::Mat pclx() const {
            cv::Mat lx, ly;
            pcLaplace(lx, ly);
            return lx;
        }

        //! get x response of pc laplace filter
        virtual cv::Mat pcly() const {
            cv::Mat lx, ly;
            pcLaplace(lx, ly);
            return ly;
        }

        //! get phase congruency laplace range
        virtual EnergyRange pcLaplaceRange() const = 0;

    };

    //! Quadrature base class helper
    template<class IT, class GT, class MT, class ET, class DT>
    class PhaseCongruencyLaplace : public PhaseCongruencyLaplaceI<ET>, public QuadratureI<IT, GT, MT, ET, DT> {
        PhaseCongruencyLaplace();
        PhaseCongruencyLaplace(const PhaseCongruencyLaplace&);
    protected:
        Range<IT> intRange_;

        PhaseCongruencyLaplace(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
            if (intRange_.lower > intRange_.upper)
                intRange_.swap();
        }

    public:
        typedef IT img_type;
        typedef GT grad_type;
        typedef MT mag_type;
        typedef ET energy_type;
        typedef DT dir_type;
        typedef DT phase_type;

        typedef Range<IT> IntensityRange;
        typedef Range<GT> GradientRange;
        typedef Range<MT> MagnitudeRange;
        typedef Range<ET> EnergyRange;
        typedef Range<DT> DirectionRange;
        typedef Range<DT> PhaseRange;


        //! get image intensity range (for single channel)
        IntensityRange intensityRange() const {
            return intRange_;
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
            ret["pclx"] = FilterData(this->pclx(), this->pcLaplaceRange());
            ret["pcly"] = FilterData(this->pcly(), this->pcLaplaceRange());
            return ret;
        }
    };

}
#endif
#endif
