#pragma once

#include <eval/cv_data_provider.hpp>
#include <eval/performance_task.hpp>
#include <opencv2/opencv.hpp>
#include <utility/value_manager.hpp>

#include <iostream>
#include <memory>

namespace lsfm {

/// @brief Extended CV data with grayscale conversion
struct CVPerformanceData : public GenericInputData {
  CVPerformanceData() = default;
  CVPerformanceData(const std::string& n, const cv::Mat& s) : GenericInputData{n}, src(s) {
    if (src.channels() == 3) {
      cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    } else {
      src_gray = src;
      cv::cvtColor(src_gray, src, cv::COLOR_GRAY2RGB);
    }
  }

  cv::Mat src{};
  cv::Mat src_gray{};
};

/// @brief Data provider for CV performance data
using CVPerformanceDataProvider = DataProvider<CVPerformanceData>;

/// @brief File-based CV performance data provider
class FileCVPerformanceDataProvider : public CVPerformanceDataProvider {
 public:
  FileCVPerformanceDataProvider(const std::string& provider_name) : CVPerformanceDataProvider(provider_name) {}

  FileCVPerformanceDataProvider(const std::filesystem::path& p, const std::string& provider_name, bool recursive = true)
      : CVPerformanceDataProvider(provider_name) {
    parse(p, recursive);
  }

  FileCVPerformanceDataProvider(const std::vector<std::filesystem::path>& f,
                                const std::string& provider_name,
                                bool recursive = true)
      : CVPerformanceDataProvider(provider_name) {
    parse(f, recursive);
  }

  virtual ~FileCVPerformanceDataProvider() = default;

  void parse(const std::vector<std::filesystem::path>& folders, bool recursive = true) {
    for (const auto& f : folders) {
      parse(f, recursive);
    }
  }

  void parse(const std::filesystem::path& folder, bool recursive = true) {
    if (!std::filesystem::exists(folder)) return;

    auto add_file = [this](const std::filesystem::path& p) {
      std::string ext = p.extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tif" || ext == ".tiff") {
        files_.push_back(p);
      }
    };

    if (recursive) {
      for (const auto& entry : std::filesystem::recursive_directory_iterator(folder)) {
        if (entry.is_regular_file()) {
          add_file(entry.path());
        }
      }
    } else {
      for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.is_regular_file()) {
          add_file(entry.path());
        }
      }
    }
  }

  bool get(CVPerformanceData& data) override {
    if (pos_ >= files_.size()) {
      return false;
    }
    cv::Mat src = cv::imread(files_[pos_].string());
    if (src.empty()) {
      ++pos_;
      return get(data);  // Skip invalid files
    }
    data = CVPerformanceData(files_[pos_].filename().string(), src);
    ++pos_;
    return true;
  }

  void rewind() override { pos_ = 0; }

  void clear() override {
    files_.clear();
    pos_ = 0;
  }

 private:
  std::size_t pos_{0};
  std::vector<std::filesystem::path> files_{};
};

/// @brief Task flags for performance tasks
constexpr int TASK_SQR = 1;
constexpr int TASK_RGB = 2;
constexpr int TASK_NO_3 = 4;
constexpr int TASK_NO_5 = 8;

/// @brief CV Performance task base class - provides clean interface for performance measurements
///
/// Supports two usage patterns:
/// 1. New style (preferred): Override prepareImpl() and runImpl() - timing handled automatically
/// 2. Legacy style: Override run(src_name, src, loops, verbose) - full control over timing
///
/// New style workflow:
/// - prepareImpl(src): Called once before timing loop - do warmup/preprocessing here
/// - runImpl(src): Called N times with automatic timing - do measured work here
///
/// The default implementation of run() calls prepareImpl() once, then runImpl() in a loop with timing.
class CVPerformanceTaskBase : public PerformanceTask<CVPerformanceData> {
 public:
  using Base = PerformanceTask<CVPerformanceData>;
  using InputData = CVPerformanceData;

  CVPerformanceTaskBase(const std::string& task_name, int flags = 0, bool verbose_flag = false)
      : Base(task_name, verbose_flag), flags_(flags) {}
  ~CVPerformanceTaskBase() override = default;

  using Base::name;

  /// @brief Check if task requires RGB input
  bool rgb() const { return (flags_ & TASK_RGB) != 0; }

  /// @brief Check if task uses squared magnitude
  bool sqr() const { return (flags_ & TASK_SQR) != 0; }

  /// @brief Get border size for task
  virtual int border() const {
    if (flags_ & TASK_NO_3) return -2;
    if (flags_ & TASK_NO_5) return -3;
    return 0;
  }

  /// @brief Access current measure for legacy compatibility
  CVPerformanceMeasure& measure_ref() { return current_measure_; }
  const CVPerformanceMeasure& measure_ref() const { return current_measure_; }

  /// @brief Legacy interface: called for saving visual results
  virtual void saveResults() {}

  /// @brief Set a parameter value by name (used by tests to configure tasks)
  virtual void value(const std::string& /*param_name*/, const lsfm::Value& /*param_value*/) {}

  /// @brief Access current data (for subclasses that need it in runImpl)
  const InputData& currentData() const { return current_data_; }

  // Bring base class prepare overloads into scope to avoid hiding
  using InputTask<InputData>::prepare;

  // Public interface for TaskRunner
  void prepare(const InputData& data) override {
    current_data_ = data;
    current_measure_ = CVPerformanceMeasure{data.name, this->name, static_cast<double>(data.src.cols),
                                            static_cast<double>(data.src.rows)};
  }

  void run(std::size_t loops) override {
    const cv::Mat& src = rgb() ? current_data_.src : current_data_.src_gray;

    // Delegate to legacy interface - allows subclasses to override either
    run(current_data_.name, src, static_cast<int>(loops), this->verbose);
  }

  void measure() override {
    // Legacy code may have written to perf_measure vector directly
    // Sync it to the measures_ map
    for (const auto& pm : perf_measure) {
      this->measures_[pm.source_name] = pm;
    }
    // Also include current_measure_ if it was used (new style)
    if (!current_measure_.durations.empty()) {
      this->measures_[current_data_.name] = current_measure_;
    }
    // Clear the legacy vector for next run
    perf_measure.clear();
  }

 protected:
  /// @brief Legacy run interface - override this for full timing control
  /// @param src_name Name of the source image
  /// @param src Input image (grayscale or RGB based on rgb() flag)
  /// @param loops Number of iterations
  /// @param verbose_flag Whether to print timing information
  virtual void run(const std::string& src_name, const cv::Mat& src, int loops, bool verbose_flag) {
    if (verbose_flag) {
      std::cout << "    Running " << this->name << " ... " << std::flush;
    }

    // Preparation/warmup phase (not timed)
    prepareImpl(src);

    // Warmup run (not timed, but ensures caches are hot)
    runImpl(src_name, src);

    // Timed runs
    for (int i = 0; i < loops; ++i) {
      std::uint64_t start = static_cast<std::uint64_t>(cv::getTickCount());
      runImpl(src_name, src);
      current_measure_.append(static_cast<std::uint64_t>(cv::getTickCount()) - start);
    }

    if (verbose_flag) {
      PerformanceResult res = current_measure_.computeResult();
      std::cout << std::setprecision(3) << res.mean << "ms" << std::endl;
    }
  }

  /// @brief New style: Override this to do preparation work (not timed)
  /// @param src Input image (grayscale or RGB based on rgb() flag)
  ///
  /// Called once before the timing loop. Use for:
  /// - Gradient/derivative computation that provides input to measured algorithm
  /// - Memory allocation that shouldn't be measured
  /// - Warmup of any caches
  virtual void prepareImpl(const cv::Mat& src) { static_cast<void>(src); }

  /// @brief New style: Override this to implement the actual task logic (timed)
  /// @param src_name Name of the source image
  /// @param src Input image
  ///
  /// Called N times with automatic timing. Only the work in this method is measured.
  virtual void runImpl(const std::string& src_name, const cv::Mat& src) {
    static_cast<void>(src_name);
    static_cast<void>(src);
  }

  /// @brief Legacy measure vector - for compatibility with old API
  /// Legacy code writes directly to this vector; it gets synced to measures_ map via measure()
  /// Note: Use `perf_measure` instead of `measure` because `measure()` is a method in the base class
  CVPerformanceMeasureVector perf_measure{};

 private:
  InputData current_data_{};
  CVPerformanceMeasure current_measure_{};
  int flags_{0};
};

using CVPerformanceTaskPtr = std::shared_ptr<CVPerformanceTaskBase>;
using CVPerformanceTaskList = std::vector<CVPerformanceTaskPtr>;

/// @brief CV Performance test runner
using CVPerformanceTest = PerformanceTest<CVPerformanceTaskBase>;
using CVPerformanceTestPtr = std::shared_ptr<CVPerformanceTest>;

// ============================================================================
// Type aliases for cleaner API
// ============================================================================

/// @brief Alias for CVPerformanceData
using PerformanceData = CVPerformanceData;

/// @brief Alias for CVPerformanceTaskBase
using PerformanceTaskDefault = CVPerformanceTaskBase;

/// @brief Alias for performance task pointer
using PerformanceTaskPtr = CVPerformanceTaskPtr;

/// @brief Alias for performance task list
using PerformanceTaskList = CVPerformanceTaskList;

/// @brief File data provider for performance tests
class FileDataProvider : public FileCVPerformanceDataProvider {
 public:
  FileDataProvider(const std::string& folder, const std::string& provider_name)
      : FileCVPerformanceDataProvider(std::filesystem::path(folder), provider_name) {}
  virtual ~FileDataProvider() = default;
};

/// @brief Data provider list type
using DataProviderList = std::vector<std::shared_ptr<DataProvider<CVPerformanceData>>>;

}  // namespace lsfm
