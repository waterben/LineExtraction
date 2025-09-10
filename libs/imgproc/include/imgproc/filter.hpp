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

#ifndef _FILTER_HPP_
#define _FILTER_HPP_
#ifdef __cplusplus

#  include <utility/range.hpp>
#  include <utility/value_manager.hpp>
#  include <opencv2/core/core.hpp>

#  include <map>


namespace lsfm {
    struct FilterData {
        FilterData(const cv::Mat &d = cv::Mat(), double lower = 0, double upper = 0) : data(d), range(lower, upper) {}

        template<class FT>
        FilterData(const cv::Mat &d, const Range<FT>& r) :
            data(d), range(static_cast<double>(r.lower), static_cast<double>(r.upper)) {}
        

        cv::Mat data;
        Range<double> range;
    };

    typedef std::map<std::string, FilterData> FilterResults;
    typedef std::pair<std::string, FilterData> FilterResult;

    //! Basic filter interface
    //! Use IT to define Image type (8Bit, 16Bit, 32Bit, or floating type float or double)
    template<class IT>
    class FilterI : public ValueManager {
        FilterI(const FilterI&);

    protected:
        FilterI() {}

    public:
        typedef IT img_type;
                
        typedef Range<IT> IntensityRange;

        virtual ~FilterI() {}
        
        //! get image intensity range (for single channel)
        virtual IntensityRange intensityRange() const = 0;

        //! process filter
        virtual void process(const cv::Mat& img) = 0;

        //! generic interface to get processed data
        virtual FilterResults results() const = 0;

        //! get name of filter
        virtual std::string name() const = 0;

    };
}
#endif
#endif
