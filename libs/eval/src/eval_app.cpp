#include <utility/eval_app.hpp>
#include <utility/high_prio.hpp>

namespace lsfm {

void EvalApp::defineArgs() {
  ConsoleApp::defineArgs();
  // clang-format off
  options_.add_options()
  ("input,i", boost::program_options::value<std::string>(&input_)->required(), "Input file or folder (required)")
  ("output,o", boost::program_options::value<std::string>(&output_), "Output file or folder")
  ("recursive,r", boost::program_options::bool_switch(&recursive_), "Enable recursive folder crawling for input")
  ("prio,p", boost::program_options::bool_switch(&run_high_prio_), "Run as high prio process")
  ("no-results", boost::program_options::bool_switch(&no_results_), "Don't write results")
  ("write-visuals", boost::program_options::bool_switch(&write_visuals_), "Write visual results")
  ("show-visuals", boost::program_options::bool_switch(&show_visuals_), "Show visual results");
  // clang-format on
}

int EvalApp::run() {
  initEval();
  if (run_high_prio_) {
    setHighPriority();
  }
  runEval();
  terminateEval();
  return 0;
}

}  // namespace lsfm
