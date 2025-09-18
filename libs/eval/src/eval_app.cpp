#include <eval/eval_app.hpp>
#include <utility/high_prio.hpp>

namespace lsfm {

void EvalApp::defineArgs() {
  ConsoleApp::defineArgs();
  opts_.add_string("input", 'i', "Input file or folder (required)", input_, true);
  opts_.add_string("output", 'o', "Output file or folder", output_);
  opts_.add_switch("recursive", 'r', "Enable recursive folder crawling for input", recursive_);
  opts_.add_switch("prio", 'p', "Run as high prio process", run_high_prio_);
  opts_.add_switch("no-results", '\0', "Don't write results", no_results_);
  opts_.add_switch("write-visuals", '\0', "Write visual results", write_visuals_);
  opts_.add_switch("show-visuals", '\0', "Show visual results", show_visuals_);
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
