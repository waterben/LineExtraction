#include "opencv2/imgproc/imgproc.hpp"
#include "performance_test.hpp"


using namespace lsfm;

struct EntryFFTCPU : public PerformanceTaskDefault {
  EntryFFTCPU() : PerformanceTaskDefault("FFT CPU") {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat tmp, padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);  // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);  // Add to the expanded another plane with zeros
    uint64 start;
    cv::dft(complexI, tmp, cv::DFT_COMPLEX_OUTPUT);
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      cv::dft(complexI, tmp, cv::DFT_COMPLEX_OUTPUT);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};


struct EntryFFTCL : public PerformanceTaskDefault {
  EntryFFTCL() : PerformanceTaskDefault("FFT CL") {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat padded, tmp2;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);  // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);  // Add to the expanded another plane with zeros
    uint64 start;

    cv::UMat in = complexI.getUMat(cv::ACCESS_READ), tmp;  // to GPU
    cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);
    tmp.copyTo(tmp2);  // to RAM

    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      in = complexI.getUMat(cv::ACCESS_READ);  // to GPU
      cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);
      tmp.copyTo(tmp2);  // to RAM

      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};

struct EntryFFTCLNT : public PerformanceTaskDefault {
  EntryFFTCLNT() : PerformanceTaskDefault("FFT CL NT") {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat padded, tmp2;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);  // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);                        // Add to the expanded another plane with zeros
    cv::UMat in = complexI.getUMat(cv::ACCESS_READ), tmp;  // to GPU
    cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);

    uint64 start;
    for (int i = 0; i != runs; ++i) {
      in = complexI.getUMat(cv::ACCESS_READ);  // to GPU
      start = cv::getTickCount();
      cv::dft(in, tmp, cv::DFT_COMPLEX_OUTPUT);
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
struct EntryFFTCuda : public PerformanceTaskDefault {
  EntryFFTCuda() : PerformanceTaskDefault("FFT Cuda") {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat padded, tmp2;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);  // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);  // Add to the expanded another plane with zeros
    uint64 start;
    cv::cuda::GpuMat in, tmp;
    in.upload(complexI);
    cv::cuda::dft(in, tmp, in.size());
    tmp.download(tmp2);
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      in.upload(complexI);
      cv::cuda::dft(in, tmp, in.size());
      tmp.download(tmp2);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};

struct EntryFFTCudaNT : public PerformanceTaskDefault {
  EntryFFTCudaNT() : PerformanceTaskDefault("FFT Cuda NT") {}

  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols);  // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);  // Add to the expanded another plane with zeros
    uint64 start;
    cv::cuda::GpuMat in, tmp;
    in.upload(complexI);
    cv::cuda::dft(in, tmp, in.size());
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      cv::cuda::dft(in, tmp, in.size());
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms"
                << std::endl;
  }
};
#endif


PerformanceTestPtr createGpuFFTPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<PerformanceTest>();
  test->name = "GPU FFT";
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

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
