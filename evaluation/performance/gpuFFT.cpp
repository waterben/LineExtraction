/// @file gpuFFT.cpp
/// @brief GPU FFT performance tests comparing CPU, OpenCL and CUDA implementations
#include "performance_test.hpp"
#include <opencv2/imgproc/imgproc.hpp>


using namespace lsfm;


// =============================================================================
// CPU FFT Performance Tasks
// =============================================================================

/// @brief CPU FFT performance task
class EntryFFTCPU : public CVPerformanceTaskBase {
  cv::Mat padded_;
  cv::Mat complexI_;
  cv::Mat tmp_;

 public:
  EntryFFTCPU() : CVPerformanceTaskBase("FFT CPU") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);
    cv::copyMakeBorder(src, padded_, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded_), cv::Mat::zeros(padded_.size(), CV_32F)};
    cv::merge(planes, 2, complexI_);
    cv::dft(complexI_, tmp_, cv::DFT_COMPLEX_OUTPUT);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    cv::dft(complexI_, tmp_, cv::DFT_COMPLEX_OUTPUT);
  }
};


// =============================================================================
// OpenCL FFT Performance Tasks
// =============================================================================

/// @brief OpenCL FFT with memory transfer performance task
class EntryFFTCL : public CVPerformanceTaskBase {
  cv::Mat complexI_;

 public:
  EntryFFTCL() : CVPerformanceTaskBase("FFT CL") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::merge(planes, 2, complexI_);
    cv::UMat in = complexI_.getUMat(cv::ACCESS_READ);
    cv::UMat tmp;
    cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat tmp2;
    tmp.copyTo(tmp2);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    cv::UMat in = complexI_.getUMat(cv::ACCESS_READ);
    cv::UMat tmp;
    cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);
    cv::Mat tmp2;
    tmp.copyTo(tmp2);
  }
};

/// @brief OpenCL FFT without memory transfer (compute only) performance task
class EntryFFTCLNT : public CVPerformanceTaskBase {
  cv::Mat complexI_;
  cv::UMat in_;
  cv::UMat tmp_;

 public:
  EntryFFTCLNT() : CVPerformanceTaskBase("FFT CL NT") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::merge(planes, 2, complexI_);
    in_ = complexI_.getUMat(cv::ACCESS_READ);
    cv::dft(in_, tmp_, cv::DFT_COMPLEX_OUTPUT);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    in_ = complexI_.getUMat(cv::ACCESS_READ);
    cv::dft(in_, tmp_, cv::DFT_COMPLEX_OUTPUT);
  }
};


#ifdef ENABLE_CUDA
// =============================================================================
// CUDA FFT Performance Tasks
// =============================================================================

/// @brief CUDA FFT with memory transfer performance task
class EntryFFTCuda : public CVPerformanceTaskBase {
  cv::Mat complexI_;
  cv::cuda::GpuMat in_;
  cv::cuda::GpuMat tmp_;

 public:
  EntryFFTCuda() : CVPerformanceTaskBase("FFT Cuda") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::merge(planes, 2, complexI_);
    in_.upload(complexI_);
    cv::cuda::dft(in_, tmp_, in_.size());
    cv::Mat tmp2;
    tmp_.download(tmp2);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    in_.upload(complexI_);
    cv::cuda::dft(in_, tmp_, in_.size());
    cv::Mat tmp2;
    tmp_.download(tmp2);
  }
};

/// @brief CUDA FFT without memory transfer (compute only) performance task
class EntryFFTCudaNT : public CVPerformanceTaskBase {
  cv::cuda::GpuMat in_;
  cv::cuda::GpuMat tmp_;

 public:
  EntryFFTCudaNT() : CVPerformanceTaskBase("FFT Cuda NT") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);
    in_.upload(complexI);
    cv::cuda::dft(in_, tmp_, in_.size());
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    cv::cuda::dft(in_, tmp_, in_.size());
  }
};
#endif


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create GPU FFT performance test with all tasks
CVPerformanceTestPtr createGpuFFTPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "GPU FFT");

  test->tasks.push_back(std::make_shared<EntryFFTCPU>());
  test->tasks.push_back(std::make_shared<EntryFFTCL>());
  test->tasks.push_back(std::make_shared<EntryFFTCLNT>());
#ifdef ENABLE_CUDA
  test->tasks.push_back(std::make_shared<EntryFFTCuda>());
  test->tasks.push_back(std::make_shared<EntryFFTCudaNT>());
#endif

  return test;
}

bool addGpuFFT() {
  addPerformanceTestCreator(createGpuFFTPerformanceTest);
  std::cout << "Added GPU FFT performance test" << std::endl;
  return true;
}

// bool gpuFFTAdded = addGpuFFT();
