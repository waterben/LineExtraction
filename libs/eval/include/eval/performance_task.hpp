#pragma once

#include <eval/cv_measure.hpp>
#include <eval/measure_task.hpp>
#include <utility/format.hpp>
#include <utility/string_table.hpp>

#include <algorithm>

namespace lsfm {

// For backward compatibility, PerformanceMeasure is CVPerformanceMeasure
// since all existing code uses CV-specific features (width/height)
// Note: Include cv_measure.hpp to get PerformanceMeasure, CVPerformanceMeasure,
// and the pure PerformanceMeasureBase (if needed without CV dependency)

/// @brief Accumulate multiple performance measures into one (backward compatibility)
inline CVPerformanceMeasure accumulateMeasures(const CVPerformanceMeasureVector& measures,
                                               const std::string& source_name = std::string(),
                                               const std::string& task_name = std::string()) {
  return accumulateCVMeasures(measures, source_name, task_name);
}

/// @brief Performance task that measures execution time
template <typename InputDataT>
class PerformanceTask : public MeasureTask<InputDataT, CVPerformanceMeasure> {
 public:
  using Base = MeasureTask<InputDataT, CVPerformanceMeasure>;
  using InputData = InputDataT;
  using Measures = CVPerformanceMeasure;

  PerformanceTask(const std::string& pt_name, bool pt_verbose = false) : Base(pt_name, pt_verbose) {}
  ~PerformanceTask() override = default;

  using Base::measures;
  using Base::name;
  using Base::prepare;
  using Base::run;
  using Base::saveVisualResults;

  /// @brief Receive accumulated performance measure of all runs of this task
  CVPerformanceMeasure accumulatedMeasure(const std::string& source_name = std::string()) const {
    CVPerformanceMeasureVector all_measures;
    for (const auto& [key, measure] : this->measures_) {
      all_measures.push_back(measure);
    }
    return accumulateMeasures(all_measures, source_name, this->name);
  }
};

/// @brief Full performance test runner based on performance tasks and data providers
template <typename PerformanceTaskT>
class PerformanceTest : public MeasureTaskRunner<PerformanceTaskT> {
 public:
  using PerformanceTaskType = PerformanceTaskT;
  using InputData = typename PerformanceTaskType::InputData;
  using Base = MeasureTaskRunner<PerformanceTaskType>;
  using DataProviderType = DataProvider<InputData>;
  using DataProviderPtrList = typename Base::DataProviderPtrList;

  PerformanceTest(DataProviderPtrList pt_data_provider,
                  const std::string& pt_name,
                  const std::string& pt_target_path = std::string(),
                  bool pt_verbose = false,
                  bool pt_visual_results = false)
      : Base(std::move(pt_data_provider), pt_name, pt_target_path, pt_verbose, pt_visual_results) {}
  ~PerformanceTest() override = default;

  using Base::data_provider;
  using Base::name;
  using Base::run;
  using Base::tasks;

  bool show_total{false};
  bool show_mean{true};
  bool show_std_dev{true};
  bool show_mega_pixel{false};

  /// @brief Generate result table from collected measures
  StringTable resultTable(bool /*fullReport*/ = false) override {
    if (this->tasks.empty()) return StringTable();

    // Collect all source names from first task
    std::vector<std::string> sources;
    for (const auto& [source_name, measure] : this->tasks.front()->measures()) {
      sources.push_back(source_name);
    }

    // Calculate rows per source (based on show* flags)
    std::size_t rows_per_source = 0;
    if (show_total) ++rows_per_source;
    if (show_mean) ++rows_per_source;
    if (show_std_dev) ++rows_per_source;
    if (rows_per_source == 0) rows_per_source = 1;  // At least one row

    // +1 for accumulated totals
    std::size_t total_rows = (sources.size() + 1) * rows_per_source + 1;  // +1 for header
    std::size_t total_cols = this->tasks.size() + 1;                      // +1 for row labels

    StringTable table(total_rows, total_cols);

    // Header row
    table(0, 0) = this->name.empty() ? "Source" : this->name;
    for (std::size_t t = 0; t < this->tasks.size(); ++t) {
      table(0, t + 1) = this->tasks[t]->name;
    }

    // Data rows per source
    std::size_t row = 1;
    for (const auto& source : sources) {
      writeMeasureRow(source, table, row, rows_per_source);
      row += rows_per_source;
    }

    // Accumulated totals
    writeTotalRow(table, row, rows_per_source);

    return table;
  }

 protected:
  void writeMeasureRow(const std::string& source, StringTable& table, std::size_t row, std::size_t rows_per_source) {
    for (std::size_t t = 0; t < this->tasks.size(); ++t) {
      const auto& measures_map = this->tasks[t]->measures();
      auto it = measures_map.find(source);
      if (it == measures_map.end()) continue;

      const CVPerformanceMeasure& pm = it->second;
      writeMeasure(pm, table, t + 1, row, rows_per_source);
    }

    // Row labels
    std::string label = source;
    if (show_mega_pixel && !this->tasks.empty()) {
      const auto& measures_map = this->tasks.front()->measures();
      auto it = measures_map.find(source);
      if (it != measures_map.end()) {
        double mpix = it->second.megaPixels();
        label += utility::format(" (%.2f MP)", mpix);
      }
    }
    std::size_t r = row;
    if (show_total) table(r++, 0) = "total:" + label;
    if (show_mean) table(r++, 0) = "mean:" + label;
    if (show_std_dev) table(r, 0) = "sdev:" + label;
  }

  void writeTotalRow(StringTable& table, std::size_t row, std::size_t rows_per_source) {
    for (std::size_t t = 0; t < this->tasks.size(); ++t) {
      CVPerformanceMeasure acc = this->tasks[t]->accumulatedMeasure("Total");
      writeMeasure(acc, table, t + 1, row, rows_per_source);
    }
    std::size_t r = row;
    if (show_total) table(r++, 0) = "total:TOTAL";
    if (show_mean) table(r++, 0) = "mean:TOTAL";
    if (show_std_dev) table(r, 0) = "sdev:TOTAL";
  }

  void writeMeasure(const CVPerformanceMeasure& pm,
                    StringTable& table,
                    std::size_t col,
                    std::size_t row,
                    std::size_t /*rows_per_source*/) {
    PerformanceResult res = pm.computeResult();
    std::size_t r = row;
    if (show_total) table(r++, col) = utility::format("%.3f", res.total);
    if (show_mean) table(r++, col) = utility::format("%.3f", res.mean);
    if (show_std_dev) table(r, col) = utility::format("%.3f", res.stddev);
  }
};

// Note: PerformanceTestPtr alias is defined in cv_performance_task.hpp for legacy compatibility

}  // namespace lsfm
