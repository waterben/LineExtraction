#ifndef _PERFORMANCE_TEST_HPP_
#define _PERFORMANCE_TEST_HPP_
#ifdef __cplusplus

#  include <utility/performance.hpp>

#  include <iomanip>
#  include <iostream>
#  include <map>
#  include <vector>


#  define ENABLE_CUDA

#  include <opencv2/core.hpp>

#  ifdef ENABLE_CUDA
#    include <opencv2/core/cuda.hpp>
#    include <opencv2/cudaarithm.hpp>
#    include <opencv2/cudafilters.hpp>
#  endif


#  if CV_MAJOR_VERSION < 3
#    include <opencv2/core/ocl.hpp>
#    include <opencv2/ocl/ocl.hpp>
#  endif

void addPerformanceTest(lsfm::PerformanceTestPtr test);

typedef std::function<lsfm::PerformanceTestPtr(const lsfm::DataProviderList&)> PerformanceTestCreator;

void addPerformanceTestCreator(PerformanceTestCreator creator);
std::vector<lsfm::PerformanceTestPtr>& getTests();
const lsfm::DataProviderList& getDefaultProvider();

#endif
#endif
