#include "performance_test.hpp"
#include <filesystem>
#include <string>
#include <utility/high_prio.hpp>

#include <functional>
#include <iostream>


using namespace lsfm;
namespace fs = std::filesystem;

std::vector<PerformanceTestPtr>& getTests() {
  static std::vector<PerformanceTestPtr> tests;
  return tests;
}

void addPerformanceTest(PerformanceTestPtr test) { getTests().push_back(test); }

const DataProviderList& getDefaultProvider() {
  static DataProviderList list;
  if (list.empty()) {
    try {
      // list.push_back(std::make_shared<FileDataProvider>("../images", "images"));
      // list.push_back(std::make_shared<FileDataProvider>("../images/small/a", "a"));
      // list.push_back(std::make_shared<FileDataProvider>("../images/small/b", "b"));
      // list.push_back(std::make_shared<FileDataProvider>("../images/small/c", "c"));
      // list.push_back(std::make_shared<FileDataProvider>("../images/Selection", "Selection"));
      list.push_back(std::make_shared<FileDataProvider>("../images/BSDS500", "BSDS500"));
      list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-Q", "MDB-Q"));
      list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-H", "MDB-H"));
      list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-F", "MDB-F"));
    } catch (std::exception& e) {
      std::cout << "Default provider parse error: " << e.what() << std::endl;
    }
  }
  return list;
}

void addPerformanceTestCreator(PerformanceTestCreator creator) {
  fs::create_directory("./results");
  fs::create_directory("./results/visual");
  fs::create_directory("./results/performance");
  getTests().emplace_back(creator(getDefaultProvider()));
}

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
    } else if (val == "-v" || val == "-verbose")
      verbose = true;
    else if (val == "-hp" || val == "-high_prio") {
      highPriority = true;
      askPrio = false;
    } else if (val == "-lp" || val == "-low_prio") {
      highPriority = false;
      askPrio = false;
    } else if (val == "-t" || val == "-transpose")
      tasksAsCols = true;
    else if (val == "-fr" || val == "-full_report")
      fullReport = true;
    else if (val == "-co" || val == "-console_out")
      printTables = true;
    else if (val == "-no_total" || val == "-nt")
      showTotal = false;
    else if (val == "-no_mean" || val == "-nm")
      showMean = false;
    else if (val == "-no_stddev" || val == "-ns")
      showStdDev = false;
    else if (val == "-no_mpix" || val == "-np")
      showMegaPixel = false;
    else if (val == "-no_csv" || val == "-nc")
      skipTableWrite = true;
    else if (val == "-visual_res" || val == "-vr")
      visualResults = true;
    else if ((val == "-r" || val == "-runs") && i < argc + 1) {
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
        std::cout << "Unkown error!" << std::endl;
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

  if (highPriority) setHighPriority();

  if (verbose) std::cout << "Number of runs: " << runs << std::endl;

  uint64 start = cv::getTickCount();

  for_each(getTests().begin(), getTests().end(), [&](PerformanceTestPtr test) {
    test->showMean = showMean;
    test->showTotal = showTotal;
    test->showMegaPixel = showMegaPixel;
    test->showStdDev = showStdDev;
    test->visualResults = visualResults;
    test->run(runs, verbose);
    StringTable result = test->resultTable(fullReport);
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
            << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s" << std::endl;

  char c;
  std::cin >> c;
  return 0;
}
