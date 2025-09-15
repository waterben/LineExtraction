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

#ifndef _OPTION_MANAGER_HPP_
#define _OPTION_MANAGER_HPP_
#ifdef __cplusplus

#include <vector>
#include <algorithm>

namespace lsfm {

    class OptionManager
    {
    public:
        struct OptionEntry {
            OptionEntry(std::string n = std::string(), double v = 0,
                std::string t = std::string(), std::string d = std::string())
                : name(n), value(v), type(t), description(d) {}

            std::string name;
            double value;

            std::string type;
            std::string description;


            template<typename T> T get() const {
                return static_cast<T>(value);
            }

            template<typename T> void set(T val) {
                value = static_cast<double>(val);
            }
        };

        typedef std::vector<OptionEntry> OptionVector;

        /**
        * Get list of all options
        */
        inline const OptionVector& getOptions() const { return options_; }

        /**
        * Get option entry by name
        */
        OptionEntry getOption(const std::string &name) const {
            OptionVector::const_iterator f = std::find_if(options_.begin(), options_.end(), [&name](const OptionEntry& e) {
                return (e.name == name);
            });
            return f != options_.end() ? *f : OptionEntry();
        }


        /**
        * Set single option by name and value
        */
        inline void setOption(const std::string &name, double value) {
            setOptionImpl(name, value);
        }

        /**
        * Set single option by option entry
        */
        inline void setOption(const OptionEntry &option) {
            setOption(option.name, option.value);
        }

        /**
        * Set multiple options by OptionVector
        */
        inline void setOptions(const OptionVector &options) {
            for_each(options.begin(), options.end(), [this](const OptionEntry& e) {
                setOption(e.name, e.value);
            });
        }
    protected:
        OptionManager(const OptionVector &opv = OptionVector()) : options_(opv) {}
        OptionVector options_;
        // default: do nothing
        virtual void setOptionImpl(const std::string &name, double value) {};
    };

}
#endif
#endif
