/// @file main.cpp
/// @brief Performance test main executable

#include "performance_test.hpp"
#include <eval/runfiles.hpp>
#include <utility/high_prio.hpp>

#include <algorithm>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <string>


namespace fs = std::filesystem;

namespace lsfm {

/// @brief Global runfiles instance for resolving data paths
std::unique_ptr<Runfiles> g_runfiles;

/// @brief Global test registry
std::vector<CVPerformanceTestPtr>& getTests() {
  static std::vector<CVPerformanceTestPtr> tests;
  return tests;
}

/// @brief Add a test directly
void addPerformanceTest(CVPerformanceTestPtr test) { getTests().push_back(test); }

/// @brief Get default data providers
/// Uses Bazel runfiles when available, falls back to relative paths otherwise.
const DataProviderList& getDefaultProvider() {
  static DataProviderList list;
  if (list.empty()) {
    try {
      if (g_runfiles && g_runfiles->isBazelRun()) {
        // =====================================================================
        // Bazel build: use runfiles for path resolution
        // =====================================================================

        // BSDS500 - from external @bsds500 repository (train + test + val)
        std::string bsds_path = g_runfiles->Rlocation("bsds500/BSDS500/data/images");
        list.push_back(std::make_shared<FileDataProvider>(bsds_path + "/train", "BSDS500-train"));
        list.push_back(std::make_shared<FileDataProvider>(bsds_path + "/test", "BSDS500-test"));
        list.push_back(std::make_shared<FileDataProvider>(bsds_path + "/val", "BSDS500-val"));

        // MDB - from local resources
        std::string mdb_path = g_runfiles->Rlocation("line_extraction/resources/datasets/MDB");
        list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-Q", "MDB-Q"));
        list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-H", "MDB-H"));
        list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-F", "MDB-F"));

      } else {
        // =====================================================================
        // CMake/standalone build: use relative paths from workspace root
        // Expected structure: resources/datasets/{BSDS500,MDB,noise}
        // =====================================================================

        // Try to find resources directory (check multiple possible locations)
        std::vector<std::string> search_paths = {
            "resources/datasets",        // From workspace root
            "../resources/datasets",     // From build/bin directory
            "../../resources/datasets",  // From deeper build directory
        };

        std::string base_path;
        for (const auto& path : search_paths) {
          if (fs::exists(path)) {
            base_path = path;
            break;
          }
        }

        if (base_path.empty()) {
          std::cout << "Warning: Could not find resources/datasets directory" << std::endl;
          std::cout << "  Expected at: resources/datasets/ (relative to working directory)" << std::endl;
          return list;
        }

        // BSDS500 - flat directory with all images combined
        std::string bsds_path = base_path + "/BSDS500";
        if (fs::exists(bsds_path) && !fs::is_empty(bsds_path)) {
          list.push_back(std::make_shared<FileDataProvider>(bsds_path, "BSDS500"));
        } else {
          std::cout << "Warning: BSDS500 not found at " << bsds_path << std::endl;
        }

        // MDB - Middlebury dataset
        std::string mdb_path = base_path + "/MDB";
        if (fs::exists(mdb_path + "/MiddEval3-Q")) {
          list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-Q", "MDB-Q"));
        }
        if (fs::exists(mdb_path + "/MiddEval3-H")) {
          list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-H", "MDB-H"));
        }
        if (fs::exists(mdb_path + "/MiddEval3-F")) {
          list.push_back(std::make_shared<FileDataProvider>(mdb_path + "/MiddEval3-F", "MDB-F"));
        }
      }

    } catch (std::exception& e) {
      std::cout << "Default provider parse error: " << e.what() << std::endl;
    }
  }
  return list;
}

/// @brief Add a test via creator function
void addPerformanceTestCreator(PerformanceTestCreator creator) {
  fs::create_directory("./results");
  fs::create_directory("./results/visual");
  fs::create_directory("./results/performance");
  getTests().emplace_back(creator(getDefaultProvider()));
}

}  // namespace lsfm

void help() {
  std::cout << "Performance test options:" << std::endl
            << "\t-help(h)\t\t\tthis information" << std::endl
            << "\t-verbose(v)\t\tverbose mode" << std::endl
            << "\t-high_prio(hp)\t\thigh priority mode" << std::endl
            << "\t-low_prio(lp)\t\tlow priority mode" << std::endl
            << "\t-transpose(t)\t\tprint tasks as columns" << std::endl
            << "\t-full_report(fr)\tprint full report" << std::endl
            << "\t-console_out(co)\tprint to console" << std::endl
            << "\t-runs(r)\t\tnumber of runs (default 10)" << std::endl
            << "\t-visual_res(vr)\t\tenable visual results" << std::endl
            << "\t-no_total(nt)\t\tno total measure" << std::endl
            << "\t-no_mean(nm)\t\tno mean measure" << std::endl
            << "\t-no_stddev(ns)\t\tno std deviation measure" << std::endl
            << "\t-no_mpix(np)\t\tno megapixel in title" << std::endl
            << "\t-no_csv(nc)\t\tskip csv output" << std::endl
            << std::endl;
}

int main(int argc, char** argv) {
  std::cout << "C++ version: " << __cplusplus << std::endl;

  // Initialize runfiles for Bazel data dependency resolution
  lsfm::g_runfiles = lsfm::Runfiles::Create(argv[0]);
  if (lsfm::g_runfiles->isBazelRun()) {
    std::cout << "Running under Bazel with runfiles support" << std::endl;
  }

  bool verbose = false;
  bool printTables = false;
  bool highPriority = false;
  bool askPrio = true;
  bool skipTableWrite = false;
  bool tasksAsCols = false;
  bool fullReport = false;
  bool showTotal = true;
  bool showMean = true;
  bool showStdDev = true;
  bool showMegaPixel = true;
  bool visualResults = false;
  int runs = 10;

  for (int i = 1; i < argc; ++i) {
    std::string val(argv[i]);
    if (val == "-h" || val == "-help") {
      help();
      return 0;
    } else if (val == "-v" || val == "-verbose") {
      verbose = true;
    } else if (val == "-hp" || val == "-high_prio") {
      highPriority = true;
      askPrio = false;
    } else if (val == "-lp" || val == "-low_prio") {
      highPriority = false;
      askPrio = false;
    } else if (val == "-t" || val == "-transpose") {
      tasksAsCols = true;
    } else if (val == "-fr" || val == "-full_report") {
      fullReport = true;
    } else if (val == "-co" || val == "-console_out") {
      printTables = true;
    } else if (val == "-no_total" || val == "-nt") {
      showTotal = false;
    } else if (val == "-no_mean" || val == "-nm") {
      showMean = false;
    } else if (val == "-no_stddev" || val == "-ns") {
      showStdDev = false;
    } else if (val == "-no_mpix" || val == "-np") {
      showMegaPixel = false;
    } else if (val == "-no_csv" || val == "-nc") {
      skipTableWrite = true;
    } else if (val == "-visual_res" || val == "-vr") {
      visualResults = true;
    } else if ((val == "-r" || val == "-runs") && i < argc + 1) {
      try {
        runs = std::stoi(std::string(argv[++i]));
        if (runs < 1) {
          std::cout << "Fixed number of runs from " << runs << " to 1!" << std::endl;
          runs = 1;
        }
      } catch (std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        help();
        return 1;
      } catch (...) {
        std::cout << "Unknown error!" << std::endl;
        help();
        return 1;
      }
    } else {
      std::cout << "Unknown options!" << std::endl << std::endl;
      help();
      return 1;
    }
  }

  if (askPrio) {
    std::cout << "Set high process priority? (y/n): ";
    char c;
    std::cin >> c;
    if (c == 'y' || c == 'Y') highPriority = true;
  }

  if (highPriority) lsfm::setHighPriority();

  if (verbose) std::cout << "Number of runs: " << runs << std::endl;

  std::uint64_t start = static_cast<std::uint64_t>(cv::getTickCount());

  std::for_each(lsfm::getTests().begin(), lsfm::getTests().end(), [&](lsfm::CVPerformanceTestPtr test) {
    // Configure test display options
    test->show_mean = showMean;
    test->show_total = showTotal;
    test->show_mega_pixel = showMegaPixel;
    test->show_std_dev = showStdDev;
    test->visual_results = visualResults;
    test->verbose = verbose;

    // Run the test
    test->run(static_cast<std::size_t>(runs));

    // Generate and output results
    lsfm::StringTable result = test->resultTable(fullReport);
    if (!tasksAsCols) {
      result = result.transpose();
      result(0, 0) = "Method";
    }
    if (printTables) std::cout << result << std::endl;
    if (!skipTableWrite) {
      result.saveCSV("./results/performance/" + test->name + ".csv");
    }
  });

  std::cout << "Total time for performance tests: "
            << static_cast<double>((static_cast<std::uint64_t>(cv::getTickCount()) - start)) /
                   static_cast<double>(cv::getTickFrequency())
            << "s" << std::endl;

  char c;
  std::cin >> c;
  return 0;
}
