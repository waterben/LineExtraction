#include "performance_test.hpp"

#include <opencv2/imgproc.hpp>

using namespace lsfm;

struct EntryConvCPU : public PerformanceTaskDefault {
    EntryConvCPU()
        : PerformanceTaskDefault("Conv CPU") {}

    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        cv::Mat tmp;
        uint64 start;
        cv::GaussianBlur(src,tmp,cv::Size(7,7),0);
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            cv::GaussianBlur(src,tmp,cv::Size(7,7),0);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};


struct EntryConvCL : public PerformanceTaskDefault {
    EntryConvCL()
        : PerformanceTaskDefault("Conv CL") {}

    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        uint64 start;
        cv::Mat tmp2;
#if CV_MAJOR_VERSION >= 3
        cv::UMat in = src.getUMat(cv::ACCESS_READ), tmp; // to GPU
        cv::GaussianBlur(in,tmp,cv::Size(7,7),0);
        tmp.copyTo(tmp2); // to RAM
#else
        cv::Ptr<cv::ocl::Filter> gauss = cv::ocl::createGaussianFilter(src.type(),src.type(),cv::Size(7,7),0);
        cv::ocl::oclMat in(src), tmp;
        gauss->apply(in, tmp);
        tmp.download(tmp2);
#endif
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
#if CV_MAJOR_VERSION >= 3
            in = src.getUMat(cv::ACCESS_READ);
            cv::GaussianBlur(in,tmp,cv::Size(7,7),0);
            tmp.copyTo(tmp2); // to RAM
#else
            in = cv::ocl::oclMat(src);
            gauss->apply(in, tmp);
            tmp.download(tmp2);
#endif
            pm.measures.push_back(cv::getTickCount() - start);

        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};

struct EntryConvCLNT : public PerformanceTaskDefault {
    EntryConvCLNT()
        : PerformanceTaskDefault("Conv CL NT") {}

    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
#if CV_MAJOR_VERSION >= 3
        cv::UMat in = src.getUMat(cv::ACCESS_READ), tmp; // to GPU
        cv::GaussianBlur(in,tmp,cv::Size(7,7),0);
#else
        cv::Ptr<cv::ocl::Filter> gauss = cv::ocl::createGaussianFilter(src.type(),src.type(),cv::Size(7,7),0);
        cv::ocl::oclMat in(src), tmp;
        gauss->apply(in, tmp);
#endif

        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
#if CV_MAJOR_VERSION >= 3
            cv::GaussianBlur(in,tmp,cv::Size(7,7),0);
#else
            gauss->apply(in, tmp);
#endif
            pm.measures.push_back(cv::getTickCount() - start);

        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};


#ifdef ENABLE_CUDA
struct EntryConvCuda : public PerformanceTaskDefault {
    EntryConvCuda()
        : PerformanceTaskDefault("Conv Cuda") {}

    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        cv::Mat tmp2;
        uint64 start;
        cv::Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(src.type(),src.type(),cv::Size(7,7),0);
        cv::cuda::GpuMat in, tmp;
        in.upload(src);
        gauss->apply(in,tmp);
        tmp.download(tmp2);
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            in.upload(src);
            gauss->apply(in,tmp);
            tmp.download(tmp2);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};

struct EntryConvCudaNT : public PerformanceTaskDefault {
    EntryConvCudaNT()
        : PerformanceTaskDefault("Conv Cuda NT") {}

    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        uint64 start;
        cv::Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(src.type(),src.type(),cv::Size(7,7),0);
        cv::cuda::GpuMat in, tmp;
        in.upload(src);
        gauss->apply(in,tmp);
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            gauss->apply(in,tmp);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};
#endif


void createGpuConvPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new PerformanceTest);
    test->name = "GPU Conv";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }

    test->tasks.push_back(PerformanceTaskPtr(new EntryConvCPU));
    test->tasks.push_back(PerformanceTaskPtr(new EntryConvCL));
    test->tasks.push_back(PerformanceTaskPtr(new EntryConvCLNT));
#ifdef ENABLE_CUDA
    test->tasks.push_back(PerformanceTaskPtr(new EntryConvCuda));
    test->tasks.push_back(PerformanceTaskPtr(new EntryConvCudaNT));
#endif
}

bool addGpuConv() {
    addPerformanceTestCreator(createGpuConvPerformanceTest);
    std::cout << "Added GPU Conv performance test" << std::endl;
    return true;
}

//bool gpuConvAdded = addGpuConv();

