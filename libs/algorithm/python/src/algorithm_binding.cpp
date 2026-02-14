/// @file algorithm_binding.cpp
/// @brief pybind11 bindings for the algorithm library.
///
/// Binds LineMerge, LineConnect, AccuracyMeasure, GroundTruthLoader,
/// search strategies, and ParamOptimizer to Python.

#include "algorithm_binding.hpp"

#include <algorithm/accuracy_measure.hpp>
#include <algorithm/ground_truth.hpp>
#include <algorithm/line_connect.hpp>
#include <algorithm/line_merge.hpp>
#include <algorithm/param_search.hpp>
#include <algorithm/search_strategy.hpp>
#include <cvnp/cvnp.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Helper: Convert ParamConfig (C++) <-> list[dict] (Python)
// ============================================================================

/// @brief Convert C++ ParamConfig to Python list of dicts.
static py::list param_config_to_python(const ParamConfig& config) {
  py::list result;
  for (const auto& nv : config) {
    py::dict d;
    d["name"] = nv.name;
    switch (nv.value.type()) {
      case Value::INT:
        d["value"] = nv.value.getInt();
        break;
      case Value::BOOL:
        d["value"] = nv.value.getBool();
        break;
      case Value::STRING:
        d["value"] = nv.value.getString();
        break;
      default:
        d["value"] = nv.value.getDouble();
        break;
    }
    result.append(d);
  }
  return result;
}

// ============================================================================
// LineMerge binding
// ============================================================================

template <class FT>
void bind_line_merge(py::module_& m, const std::string& suffix) {
  using LM = LineMerge<FT>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "LineMerge" + suffix;

  py::class_<LM>(m, cls.c_str(),
                 ("Merge collinear or near-collinear line segments" + suffix +
                  ".\n\n"
                  "Two segments are merge candidates when their endpoint distance,\n"
                  "angle difference, perpendicular distance, and parallel gap are\n"
                  "all below the configured thresholds.")
                     .c_str())
      .def(py::init<>(), "Construct with default parameters.")
      .def(py::init<FT, FT, FT, FT, MergeType>(), py::arg("max_dist") = FT(20), py::arg("angle_error") = FT(5),
           py::arg("distance_error") = FT(3), py::arg("parallel_error") = FT(10),
           py::arg("merge_type") = MergeType::STANDARD, "Construct with explicit parameters.")
      .def(
          "merge_lines",
          [](const LM& self, const std::vector<LST>& input) {
            std::vector<LST> output;
            self.merge_lines(input, output);
            return output;
          },
          py::arg("input"),
          "Merge collinear segments.\n\n"
          "Args:\n"
          "    input: Input line segments.\n\n"
          "Returns:\n"
          "    List of merged line segments.")
      .def("__repr__", [cls](const LM&) { return "<" + cls + ">"; });
}

// ============================================================================
// LineConnect binding
// ============================================================================

template <class FT>
void bind_line_connect(py::module_& m, const std::string& suffix) {
  using LC = LineConnect<FT>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "LineConnect" + suffix;

  py::class_<LC>(m, cls.c_str(),
                 ("Connect nearby line segments using gradient magnitude" + suffix +
                  ".\n\n"
                  "Finds pairs of nearby segment endpoints and connects them if\n"
                  "the gradient response along the connecting path is strong enough.")
                     .c_str())
      .def(py::init<>(), "Construct with default parameters.")
      .def(py::init<FT, FT, FT>(), py::arg("max_radius") = FT(15), py::arg("accuracy") = FT(2),
           py::arg("threshold") = FT(10), "Construct with explicit parameters.")
      .def(
          "connect_lines",
          [](const LC& self, const std::vector<LST>& input, const cv::Mat& magnitude) {
            std::vector<LST> output;
            self.connect_lines(input, output, magnitude);
            return output;
          },
          py::arg("input"), py::arg("magnitude"),
          "Connect segments using gradient magnitude map.\n\n"
          "Args:\n"
          "    input: Input line segments.\n"
          "    magnitude: Gradient magnitude image (float32/float64).\n\n"
          "Returns:\n"
          "    List of connected line segments.")
      .def("__repr__", [cls](const LC&) { return "<" + cls + ">"; });
}

// ============================================================================
// AccuracyMeasure binding
// ============================================================================

template <class FT>
void bind_accuracy_measure(py::module_& m, const std::string& suffix) {
  using AM = AccuracyMeasure<FT>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "AccuracyMeasure" + suffix;

  py::class_<AM>(m, cls.c_str(),
                 ("Accuracy metrics for line segment detection" + suffix +
                  ".\n\n"
                  "Matches detected segments to ground truth using endpoint\n"
                  "distance and computes precision, recall, F1, and sAP.")
                     .c_str())
      .def(py::init<double>(), py::arg("threshold") = 5.0, "Construct with matching threshold in pixels.")
      .def_property("threshold", &AM::threshold, &AM::set_threshold, "Matching threshold in pixels.")
      .def(
          "evaluate",
          [](const AM& self, const std::vector<LST>& detected, const std::vector<LST>& ground_truth) {
            return self.evaluate(detected, ground_truth);
          },
          py::arg("detected"), py::arg("ground_truth"),
          "Evaluate detected vs ground truth.\n\n"
          "Returns:\n"
          "    AccuracyResult with precision, recall, F1.")
      .def(
          "structural_ap",
          [](const AM& self, const std::vector<LST>& detected, const std::vector<LST>& ground_truth,
             const std::vector<double>& thresholds) { return self.structural_ap(detected, ground_truth, thresholds); },
          py::arg("detected"), py::arg("ground_truth"), py::arg("thresholds") = std::vector<double>{5, 10, 15},
          "Compute structural AP averaging F1 across thresholds.")
      .def("__repr__",
           [cls](const AM& self) { return "<" + cls + "(threshold=" + std::to_string(self.threshold()) + ")>"; });
}

// ============================================================================
// GroundTruth binding
// ============================================================================

void bind_ground_truth(py::module_& m) {
  py::class_<GroundTruthEntry>(m, "GroundTruthEntry",
                               "Ground truth entry for one image.\n\n"
                               "Contains image name and ground truth line segments.")
      .def(py::init<>())
      .def_readwrite("image_name", &GroundTruthEntry::image_name, "Image file name.")
      .def_readwrite("segments", &GroundTruthEntry::segments, "List of ground truth LineSegment instances.")
      .def("__repr__", [](const GroundTruthEntry& e) {
        return "<GroundTruthEntry('" + e.image_name + "', " + std::to_string(e.segments.size()) + " segments)>";
      });

  py::class_<GroundTruthLoader>(m, "GroundTruthLoader", "Load/save ground truth annotations in CSV format.")
      .def_static("load_csv", &GroundTruthLoader::load_csv, py::arg("csv_path"),
                  "Load ground truth from CSV file.\n\n"
                  "CSV format: image_name,x1,y1,x2,y2\n\n"
                  "Returns:\n"
                  "    List of GroundTruthEntry objects.")
      .def_static("save_csv", &GroundTruthLoader::save_csv, py::arg("csv_path"), py::arg("entries"),
                  "Save ground truth entries to CSV file.")
      .def_static("make_entry", &GroundTruthLoader::make_entry, py::arg("image_name"), py::arg("segments"),
                  "Create a GroundTruthEntry from image name and segments.");
}

// ============================================================================
// SearchStrategy binding
// ============================================================================

void bind_search_strategy(py::module_& m) {
  // AccuracyResult (bound once, used by all AccuracyMeasure variants)
  py::class_<AccuracyResult>(m, "AccuracyResult",
                             "Result of accuracy evaluation.\n\n"
                             "Contains precision, recall, F1, and match counts.")
      .def(py::init<>())
      .def_readwrite("precision", &AccuracyResult::precision, "Precision score.")
      .def_readwrite("recall", &AccuracyResult::recall, "Recall score.")
      .def_readwrite("f1", &AccuracyResult::f1, "F1 score.")
      .def_readwrite("true_positives", &AccuracyResult::true_positives)
      .def_readwrite("false_positives", &AccuracyResult::false_positives)
      .def_readwrite("false_negatives", &AccuracyResult::false_negatives)
      .def("__repr__", [](const AccuracyResult& r) {
        std::ostringstream os;
        os << "AccuracyResult(precision=" << r.precision << ", recall=" << r.recall << ", f1=" << r.f1
           << ", tp=" << r.true_positives << ", fp=" << r.false_positives << ", fn=" << r.false_negatives << ")";
        return os.str();
      });

  // ParamRange
  py::class_<ParamRange>(m, "ParamRange", "Defines a parameter's search range.")
      .def(py::init<const std::string&, double, double, double>(), py::arg("name"), py::arg("min_val"),
           py::arg("max_val"), py::arg("step") = 1.0, "Construct a floating-point parameter range.")
      .def_static("make_int", &ParamRange::make_int, py::arg("name"), py::arg("min_val"), py::arg("max_val"),
                  py::arg("step") = 1, "Create an integer parameter range.")
      .def_static("make_bool", &ParamRange::make_bool, py::arg("name"), "Create a boolean parameter range.")
      .def_readwrite("name", &ParamRange::name)
      .def_readwrite("min_val", &ParamRange::min_val)
      .def_readwrite("max_val", &ParamRange::max_val)
      .def_readwrite("step", &ParamRange::step)
      .def("__repr__", [](const ParamRange& r) {
        std::ostringstream os;
        os << "ParamRange('" << r.name << "', " << r.min_val << ", " << r.max_val << ", step=" << r.step << ")";
        return os.str();
      });

  // MergeType enum
  py::enum_<MergeType>(m, "MergeType", "Merge strategy for LineMerge.")
      .value("STANDARD", MergeType::STANDARD, "Use furthest endpoints.")
      .value("AVG", MergeType::AVG, "Average endpoint positions.");

  // OptimMetric enum
  py::enum_<OptimMetric>(m, "OptimMetric", "Optimization metric for ParamOptimizer.")
      .value("F1", OptimMetric::F1, "F1 score (harmonic mean of precision/recall).")
      .value("PRECISION", OptimMetric::PRECISION, "Precision only.")
      .value("RECALL", OptimMetric::RECALL, "Recall only.");

  // SearchStrategy base class (abstract — not directly constructible)
  py::class_<SearchStrategy>(m, "SearchStrategy", "Abstract base class for search strategies.");

  // GridSearchStrategy
  py::class_<GridSearchStrategy, SearchStrategy>(m, "GridSearchStrategy",
                                                 "Grid search: exhaustive Cartesian product of parameter values.")
      .def(py::init<>(), "Construct grid search strategy.")
      .def(
          "generate",
          [](GridSearchStrategy& self, const SearchSpace& space) {
            auto configs = self.generate(space);
            py::list result;
            for (const auto& config : configs) {
              result.append(param_config_to_python(config));
            }
            return result;
          },
          py::arg("space"),
          "Generate all parameter configurations.\n\n"
          "Returns:\n"
          "    List of configurations, each a list of {'name': str, 'value': ...} dicts.")
      .def("__repr__", [](const GridSearchStrategy&) { return "<GridSearchStrategy>"; });

  // RandomSearchStrategy
  py::class_<RandomSearchStrategy, SearchStrategy>(m, "RandomSearchStrategy",
                                                   "Random search: uniform random sampling of parameter space.")
      .def(py::init<int, unsigned int>(), py::arg("num_samples") = 100, py::arg("seed") = 42,
           "Construct random search strategy.\n\n"
           "Args:\n"
           "    num_samples: Number of random configurations to generate.\n"
           "    seed: Random seed for reproducibility.")
      .def(
          "generate",
          [](RandomSearchStrategy& self, const SearchSpace& space) {
            auto configs = self.generate(space);
            py::list result;
            for (const auto& config : configs) {
              result.append(param_config_to_python(config));
            }
            return result;
          },
          py::arg("space"), "Generate random parameter configurations.")
      .def("__repr__", [](const RandomSearchStrategy& s) {
        return "<RandomSearchStrategy(n=" + std::to_string(s.n_samples()) + ")>";
      });
}

// ============================================================================
// ParamOptimizer binding
// ============================================================================

void bind_param_optimizer(py::module_& m) {
  using LST = LineSegment<double, Vec2>;

  // EvalResult — params converted to Python dicts
  py::class_<EvalResult>(m, "EvalResult", "Result of evaluating one parameter configuration.")
      .def(py::init<>())
      .def_property_readonly(
          "params", [](const EvalResult& r) { return param_config_to_python(r.params); },
          "Parameter configuration as list of dicts.")
      .def_readwrite("accuracy", &EvalResult::accuracy)
      .def_readwrite("score", &EvalResult::score)
      .def("__repr__", [](const EvalResult& r) { return "<EvalResult(score=" + std::to_string(r.score) + ")>"; });

  // SearchResult — best_params converted to Python dicts
  py::class_<SearchResult>(m, "SearchResult", "Result of a parameter optimization run.")
      .def(py::init<>())
      .def_property_readonly(
          "best_params", [](const SearchResult& r) { return param_config_to_python(r.best_params); },
          "Best parameters as list of dicts.")
      .def_readwrite("best_score", &SearchResult::best_score)
      .def_readwrite("total_configs", &SearchResult::total_configs)
      .def_readwrite("all_results", &SearchResult::all_results)
      .def("top_n", &SearchResult::top_n, py::arg("n") = 10, "Get top N results sorted by score descending.")
      .def("__repr__", [](const SearchResult& r) {
        return "<SearchResult(best_score=" + std::to_string(r.best_score) +
               ", total=" + std::to_string(r.total_configs) + ")>";
      });

  // ParamOptimizer — detect_fn receives Python dicts for params
  py::class_<ParamOptimizer>(m, "ParamOptimizer",
                             "Parameter optimizer for line segment detection.\n\n"
                             "Evaluates detection functions across a parameter search\n"
                             "space using accuracy metrics (F1, precision, recall).")
      .def(py::init<OptimMetric, double, bool>(), py::arg("metric") = OptimMetric::F1, py::arg("match_threshold") = 5.0,
           py::arg("verbose") = false,
           "Construct parameter optimizer.\n\n"
           "Args:\n"
           "    metric: Optimization metric (F1, PRECISION, RECALL).\n"
           "    match_threshold: Endpoint distance threshold for matching.\n"
           "    verbose: Enable verbose output.")
      .def(
          "optimize",
          [](ParamOptimizer& self, SearchStrategy& strategy, const SearchSpace& space,
             const std::vector<std::pair<std::string, cv::Mat>>& images,
             const std::vector<GroundTruthEntry>& ground_truth, const py::function& py_detect_fn,
             const py::object& py_progress) {
            // Wrap detect_fn: convert C++ ParamConfig to Python dicts
            auto detect_fn = [&py_detect_fn](const cv::Mat& src, const ParamConfig& params) {
              py::list py_params = param_config_to_python(params);
              py::object result = py_detect_fn(src, py_params);
              return result.cast<std::vector<LST>>();
            };

            ProgressCallback cb = nullptr;
            if (!py_progress.is_none()) {
              auto progress_fn = py_progress.cast<std::function<bool(int, int, double)>>();
              cb = [&progress_fn](int step, int total, double score) -> bool {
                return progress_fn(step, total, score);
              };
            }
            return self.optimize(strategy, space, images, ground_truth, detect_fn, cb);
          },
          py::arg("strategy"), py::arg("space"), py::arg("images"), py::arg("ground_truth"), py::arg("detect_fn"),
          py::arg("progress") = py::none(),
          "Run parameter optimization.\n\n"
          "Args:\n"
          "    strategy: Search strategy (GridSearchStrategy or RandomSearchStrategy).\n"
          "    space: List of ParamRange defining the search space.\n"
          "    images: List of (name, image) tuples.\n"
          "    ground_truth: List of GroundTruthEntry objects.\n"
          "    detect_fn: Detection function (image, params) -> list[LineSegment].\n"
          "        params is a list of {'name': str, 'value': float/int/bool} dicts.\n"
          "    progress: Optional callback (step, total, score) -> bool.\n\n"
          "Returns:\n"
          "    SearchResult with best parameters and scores.")
      .def("__repr__", [](const ParamOptimizer&) { return "<ParamOptimizer>"; });
}

// ============================================================================
// Combined binding
// ============================================================================

template <class FT>
void bind_algorithm(py::module_& m, const std::string& suffix) {
  bind_line_merge<FT>(m, suffix);
  bind_line_connect<FT>(m, suffix);
  bind_accuracy_measure<FT>(m, suffix);
}

// Explicit template instantiations
template void bind_line_merge<float>(py::module_&, const std::string&);
template void bind_line_merge<double>(py::module_&, const std::string&);
template void bind_line_connect<float>(py::module_&, const std::string&);
template void bind_line_connect<double>(py::module_&, const std::string&);
template void bind_accuracy_measure<float>(py::module_&, const std::string&);
template void bind_accuracy_measure<double>(py::module_&, const std::string&);
template void bind_algorithm<float>(py::module_&, const std::string&);
template void bind_algorithm<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
