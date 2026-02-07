/// @file performance_test.hpp
/// @brief Common header for performance test executables

#pragma once

#include <eval/cv_performance_task.hpp>
#include <opencv2/core.hpp>

#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
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

namespace lsfm {

/// @brief Function type for creating performance tests
using PerformanceTestCreator = std::function<CVPerformanceTestPtr(const DataProviderList&)>;

/// @brief Add a performance test directly
void addPerformanceTest(CVPerformanceTestPtr test);

/// @brief Add a performance test creator function
void addPerformanceTestCreator(PerformanceTestCreator creator);

/// @brief Get the list of registered performance tests
std::vector<CVPerformanceTestPtr>& getTests();

/// @brief Get the default data provider list
const DataProviderList& getDefaultProvider();

}  // namespace lsfm
