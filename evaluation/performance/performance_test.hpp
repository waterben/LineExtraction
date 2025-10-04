#pragma once

#include <opencv2/core.hpp>
#include <utility/performance.hpp>

#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

// Detect OpenCV CUDA module availability via header presence
#if defined(__has_include)
#  if __has_include(<opencv2/core/cuda.hpp>) && \
        __has_include(<opencv2/cudaarithm.hpp>) && \
        __has_include(<opencv2/cudafilters.hpp>)
#    define ENABLE_CUDA 1
#  endif
#endif

#ifdef ENABLE_CUDA
#  include <opencv2/core/cuda.hpp>
#  include <opencv2/cudaarithm.hpp>
#  include <opencv2/cudafilters.hpp>
#endif


#if CV_MAJOR_VERSION < 3
#  include <opencv2/core/ocl.hpp>
#  include <opencv2/ocl/ocl.hpp>
#endif

void addPerformanceTest(lsfm::PerformanceTestPtr test);

typedef std::function<lsfm::PerformanceTestPtr(const lsfm::DataProviderList&)> PerformanceTestCreator;

void addPerformanceTestCreator(PerformanceTestCreator creator);
std::vector<lsfm::PerformanceTestPtr>& getTests();
const lsfm::DataProviderList& getDefaultProvider();
