#include "performance_test.hpp"
#include <opencv2/imgproc.hpp>


using namespace lsfm;

template <int KS>
struct EntryConvCPU : public PerformanceTaskDefault {
  EntryConvCPU() : PerformanceTaskDefault("Conv CPU " + std::to_string(KS)) {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat tmp;
    uint64 start = 0;
    cv::GaussianBlur(src, tmp, cv::Size(KS, KS), 0);
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      cv::GaussianBlur(src, tmp, cv::Size(KS, KS), 0);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};


template <int KS>
struct EntryConvCL : public PerformanceTaskDefault {
  EntryConvCL() : PerformanceTaskDefault("Conv CL " + std::to_string(KS)) {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    cv::Mat tmp2;
    cv::UMat in = src.getUMat(cv::ACCESS_READ), tmp;  // to GPU
    cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);
    tmp.copyTo(tmp2);  // to RAM

    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();

      in = src.getUMat(cv::ACCESS_READ);
      cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);
      tmp.copyTo(tmp2);  // to RAM

      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};

template <int KS>
struct EntryConvCLNT : public PerformanceTaskDefault {
  EntryConvCLNT() : PerformanceTaskDefault("Conv CL NT " + std::to_string(KS)) {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::UMat in = src.getUMat(cv::ACCESS_READ), tmp;  // to GPU
    cv::Mat tmp2;
    cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);
    tmp.copyTo(tmp2);  // to RAM
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      in = src.getUMat(cv::ACCESS_READ);
      start = cv::getTickCount();
      cv::GaussianBlur(in, tmp, cv::Size(KS, KS), 0);

      pm.measures.push_back(cv::getTickCount() - start);
      tmp.copyTo(tmp2);  // to RAM
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};


#ifdef ENABLE_CUDA
template <int KS>
struct EntryConvCuda : public PerformanceTaskDefault {
  EntryConvCuda() : PerformanceTaskDefault("Conv Cuda " + std::to_string(KS)) {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat tmp2;
    uint64 start = 0;
    cv::Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(src.type(), src.type(), cv::Size(KS, KS), 0);
    cv::cuda::GpuMat in, tmp;
    in.upload(src);
    gauss->apply(in, tmp);
    tmp.download(tmp2);
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      in.upload(src);
      gauss->apply(in, tmp);
      tmp.download(tmp2);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};

template <int KS>
struct EntryConvCudaNT : public PerformanceTaskDefault {
  EntryConvCudaNT() : PerformanceTaskDefault("Conv Cuda NT " + std::to_string(KS)) {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    cv::Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(src.type(), src.type(), cv::Size(KS, KS), 0);
    cv::cuda::GpuMat in, tmp;
    in.upload(src);
    gauss->apply(in, tmp);
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      gauss->apply(in, tmp);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};
#endif


PerformanceTestPtr createGpuConvPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<PerformanceTest>();
  test->name = "GPU Conv";
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

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
