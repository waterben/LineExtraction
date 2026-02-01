/// @file gpuConv.cpp
/// @brief GPU Convolution performance tests comparing CPU, OpenCL and CUDA implementations
#include "performance_test.hpp"
#include <opencv2/imgproc.hpp>


using namespace lsfm;


// =============================================================================
// CPU Convolution Performance Tasks
// =============================================================================

/// @brief CPU Gaussian convolution performance task
template <int KS>
class EntryConvCPU : public CVPerformanceTaskBase {
  cv::Mat tmp_{};

 public:
  EntryConvCPU() : CVPerformanceTaskBase("Conv CPU " + std::to_string(KS)), tmp_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override { cv::GaussianBlur(src, tmp_, cv::Size(KS, KS), 0); }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override {
    cv::GaussianBlur(src, tmp_, cv::Size(KS, KS), 0);
  }
};


// =============================================================================
// OpenCL Convolution Performance Tasks
// =============================================================================

/// @brief OpenCL Gaussian convolution with memory transfer performance task
template <int KS>
class EntryConvCL : public CVPerformanceTaskBase {
 public:
  EntryConvCL() : CVPerformanceTaskBase("Conv CL " + std::to_string(KS)) {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    cv::UMat in = src.getUMat(cv::ACCESS_READ);
    cv::UMat tmp;
    cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);
    cv::Mat tmp2;
    tmp.copyTo(tmp2);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override {
    cv::UMat in = src.getUMat(cv::ACCESS_READ);
    cv::UMat tmp;
    cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);
    cv::Mat tmp2;
    tmp.copyTo(tmp2);
  }
};

/// @brief OpenCL Gaussian convolution without memory transfer (compute only) performance task
template <int KS>
class EntryConvCLNT : public CVPerformanceTaskBase {
  cv::UMat in_{};
  cv::UMat tmp_{};

 public:
  EntryConvCLNT() : CVPerformanceTaskBase("Conv CL NT " + std::to_string(KS)), in_(), tmp_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    in_ = src.getUMat(cv::ACCESS_READ);
    cv::GaussianBlur(in_, tmp_, cv::Size(KS, KS), 0);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override {
    in_ = src.getUMat(cv::ACCESS_READ);
    cv::GaussianBlur(in_, tmp_, cv::Size(KS, KS), 0);
  }
};


#ifdef ENABLE_CUDA
// =============================================================================
// CUDA Convolution Performance Tasks
// =============================================================================

/// @brief CUDA Gaussian convolution with memory transfer performance task
template <int KS>
class EntryConvCuda : public CVPerformanceTaskBase {
  cv::Ptr<cv::cuda::Filter> gauss_;
  cv::cuda::GpuMat in_;
  cv::cuda::GpuMat tmp_;

 public:
  EntryConvCuda() : CVPerformanceTaskBase("Conv Cuda " + std::to_string(KS)) {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    gauss_ = cv::cuda::createGaussianFilter(src.type(), src.type(), cv::Size(KS, KS), 0);
    in_.upload(src);
    gauss_->apply(in_, tmp_);
    cv::Mat tmp2;
    tmp_.download(tmp2);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override {
    in_.upload(src);
    gauss_->apply(in_, tmp_);
    cv::Mat tmp2;
    tmp_.download(tmp2);
  }
};

/// @brief CUDA Gaussian convolution without memory transfer (compute only) performance task
template <int KS>
class EntryConvCudaNT : public CVPerformanceTaskBase {
  cv::Ptr<cv::cuda::Filter> gauss_;
  cv::cuda::GpuMat in_;
  cv::cuda::GpuMat tmp_;

 public:
  EntryConvCudaNT() : CVPerformanceTaskBase("Conv Cuda NT " + std::to_string(KS)) {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    gauss_ = cv::cuda::createGaussianFilter(src.type(), src.type(), cv::Size(KS, KS), 0);
    in_.upload(src);
    gauss_->apply(in_, tmp_);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override { gauss_->apply(in_, tmp_); }
};
#endif


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create GPU convolution performance test with all tasks
CVPerformanceTestPtr createGpuConvPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "GPU Conv");

  test->tasks.push_back(std::make_shared<EntryConvCPU<3>>());
  test->tasks.push_back(std::make_shared<EntryConvCPU<7>>());
  test->tasks.push_back(std::make_shared<EntryConvCPU<15>>());
  test->tasks.push_back(std::make_shared<EntryConvCPU<31>>());
  test->tasks.push_back(std::make_shared<EntryConvCL<3>>());
  test->tasks.push_back(std::make_shared<EntryConvCL<7>>());
  test->tasks.push_back(std::make_shared<EntryConvCL<15>>());
  test->tasks.push_back(std::make_shared<EntryConvCL<31>>());

#ifdef ENABLE_CUDA
  test->tasks.push_back(std::make_shared<EntryConvCuda<3>>());
  test->tasks.push_back(std::make_shared<EntryConvCuda<7>>());
  test->tasks.push_back(std::make_shared<EntryConvCuda<15>>());
  test->tasks.push_back(std::make_shared<EntryConvCuda<31>>());
#endif

  test->tasks.push_back(std::make_shared<EntryConvCLNT<3>>());
  test->tasks.push_back(std::make_shared<EntryConvCLNT<7>>());
  test->tasks.push_back(std::make_shared<EntryConvCLNT<15>>());
  test->tasks.push_back(std::make_shared<EntryConvCLNT<31>>());

#ifdef ENABLE_CUDA
  test->tasks.push_back(std::make_shared<EntryConvCudaNT<3>>());
  test->tasks.push_back(std::make_shared<EntryConvCudaNT<7>>());
  test->tasks.push_back(std::make_shared<EntryConvCudaNT<15>>());
  test->tasks.push_back(std::make_shared<EntryConvCudaNT<31>>());
#endif

  return test;
}

bool addGpuConv() {
  addPerformanceTestCreator(createGpuConvPerformanceTest);
  std::cout << "Added GPU Conv performance test" << std::endl;
  return true;
}

// bool gpuConvAdded = addGpuConv();
