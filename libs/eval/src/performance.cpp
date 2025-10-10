#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/format.hpp>
#include <utility/performance.hpp>

#include <iostream>


namespace lsfm {

PerformanceData::PerformanceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), src_gray() {
  if (src.channels() == 3)
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
  else {
    src_gray = src;
    cv::cvtColor(src_gray, src, CV_GRAY2RGB);
  }
}

PerformanceResult PerformanceMeasure::computeResult(const std::vector<uint64>& data) {
  PerformanceResult ret;
  if (data.empty()) return ret;
  uint64 sum = 0, sqrSum = 0;
  for_each(data.begin(), data.end(), [&](uint64 measure) {
    sum += measure;
    sqrSum += measure * measure;
  });
  const double tickFrequency = cv::getTickFrequency();
  const double count = static_cast<double>(data.size());
  ret.total = static_cast<double>(sum * 1000) / tickFrequency;
  ret.mean = static_cast<double>(sum) / count;
  ret.stddev = std::sqrt(static_cast<double>(sqrSum) / count - ret.mean * ret.mean);
  ret.mean = (ret.mean * 1000) / tickFrequency;
  ret.stddev = (ret.stddev * 1000) / tickFrequency;
  return ret;
}

PerformanceMeasure PerformanceTaskBase::accumulatedMeasure(const PerformanceMeasureVector& measure,
                                                           const std::string sourceName,
                                                           const std::string filterName) {
  PerformanceMeasure ret(sourceName, filterName);
  if (measure.empty()) return ret;

  for_each(measure.begin(), measure.end(), [&](const PerformanceMeasure& pd) {
    ret.height += pd.height;
    ret.width += pd.width;
    ret.measures.insert(ret.measures.end(), pd.measures.begin(), pd.measures.end());
  });
  const double count = static_cast<double>(measure.size());
  ret.width /= count;
  ret.height /= count;
  return ret;
}

void PerformanceTaskDefault::run(const TaskData& data, int loops, bool verbose) {
  const PerformanceData& pdata = static_cast<const PerformanceData&>(data);
  run(pdata.name, this->rgb() ? pdata.src : pdata.src_gray, loops, verbose);
}

std::unique_ptr<TaskData> PerformanceTest::prepareTaskData(const std::string& src_name, cv::Mat& src) {
  prepareSource(src);
  return std::make_unique<PerformanceData>(src_name, src);
}

void PerformanceTest::run(int runs, bool verbose) {
  std::cout << "Starting performance test: " << name << std::endl;
  const int64 start = cv::getTickCount();
  results_.clear();
  providerRecords_ = 0;
  std::string src_name;
  cv::Mat src;
  // make sure data is at beginning
  for_each(data.begin(), data.end(), [&](DataProviderPtr& d) { d->rewind(); });

  size_t pos = 0;
  while (pos < data.size()) {
    // make sure task data is empty
    for_each(tasks.begin(), tasks.end(), [&](PerformanceTaskPtr& t) { t->measure.clear(); });

    while (data[pos]->get(src_name, src)) {
      try {
        if (verbose) std::cout << "  Process source: " << src_name << std::endl;
        auto taskData = prepareTaskData(src_name, src);
        ++providerRecords_;
        for_each(tasks.begin(), tasks.end(), [&, this](PerformanceTaskPtr& task) {
          task->run(*taskData, runs, verbose);
          if (this->visualResults) task->saveResults(verbose);
        });
      } catch (std::exception& e) {
        std::cout << "\t\tException occured: " << e.what() << std::endl;
      } catch (...) {
        std::cout << "\t\tUnknown exception occured!" << std::endl;
      }
    }
    for_each(tasks.begin(), tasks.end(), [&](PerformanceTaskPtr& t) {
      results_.push_back(DataProviderTaskMeasure(data[pos]->name, t->name, t->measure));
    });
    ++pos;
  }

  std::cout << "Performance test " << name
            << " done: " << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s"
            << std::endl;
}

void PerformanceTest::writeMeasure(const PerformanceMeasure& pm, StringTable& StringTable, size_t col, size_t row) {
  std::string measure_name;
  if (col == 1) {
    measure_name = pm.sourceName;
    if (showMegaPixel) measure_name += utility::format(" (%.2f)", (pm.width * pm.height));
  }

  PerformanceResult res = pm.computeResult();
  if (showTotal) {
    if (col == 1) StringTable(row, 0) = "total:" + measure_name;
    StringTable(row++, col) = utility::format("%.3f", res.total);
  }
  if (showMean) {
    if (col == 1) StringTable(row, 0) = "mean:" + measure_name;
    StringTable(row++, col) = utility::format("%.3f", res.mean);
  }
  if (showStdDev) {
    if (col == 1) StringTable(row, 0) = "sdev:" + measure_name;
    StringTable(row, col) = utility::format("%.3f", res.stddev);
  }
}

StringTable PerformanceTest::resultTable(bool fullReport) {
  const size_t rows_per_measure = static_cast<size_t>(showTotal ? 1 : 0) + static_cast<size_t>(showMean ? 1 : 0) +
                                  static_cast<size_t>(showStdDev ? 1 : 0);
  // write tasks as cols, since it is much less than sources for full reports
  size_t cols = tasks.size() + 1;
  size_t rows = data.size() * rows_per_measure + 1;
  if (fullReport) rows += providerRecords_ * rows_per_measure;

  StringTable table(rows, cols);
  table(0, 0) = "Data";

  size_t col = 1;
  for_each(tasks.begin(), tasks.end(), [&](const PerformanceTaskPtr& e) { table(0, col++) = e->name; });

  col = 1;
  size_t row = 1;
  // in results we have row by row since each task is a col and the tasks are always written in sequence
  for_each(results_.begin(), results_.end(), [&](const DataProviderTaskMeasure& provider_data) {
    PerformanceMeasure accumulated_pm = provider_data.accumulatedResult();
    writeMeasure(accumulated_pm, table, col, row);
    if (fullReport) {
      for_each(provider_data.results.begin(), provider_data.results.end(),
               [&](const PerformanceMeasure& measure_pm) { writeMeasure(measure_pm, table, col, row); });
    }
    ++col;
    if (col >= cols) {
      row += rows_per_measure;
      col = 1;
    }
  });
  return table;
}

}  // namespace lsfm
