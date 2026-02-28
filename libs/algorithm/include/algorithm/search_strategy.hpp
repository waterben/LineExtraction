//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file search_strategy.hpp
/// @brief Parameter search strategies for optimization.
///
/// Provides grid search and random search strategies that enumerate
/// parameter combinations for use with ParamOptimizer.

#pragma once

#include <utility/value.hpp>
#include <utility/value_manager.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Defines a parameter range for search.
struct ParamRange {
  std::string name{};              ///< Parameter name (as registered in ValueManager).
  Value::Type type{Value::FLOAT};  ///< Parameter type.
  double min_val{0};               ///< Minimum value (inclusive).
  double max_val{1};               ///< Maximum value (inclusive).
  double step{0.1};                ///< Step size for grid search.

  ParamRange() = default;

  /// @brief Construct a float/double parameter range.
  /// @param n Parameter name.
  /// @param lo Minimum value.
  /// @param hi Maximum value.
  /// @param s Step size for grid search.
  ParamRange(std::string n, double lo, double hi, double s)
      : name(std::move(n)), type(Value::FLOAT), min_val(lo), max_val(hi), step(s) {}

  /// @brief Construct an integer parameter range.
  /// @param n Parameter name.
  /// @param lo Minimum value.
  /// @param hi Maximum value.
  /// @param s Step size (default 1).
  static ParamRange make_int(const std::string& n, int lo, int hi, int s = 1) {
    ParamRange r;
    r.name = n;
    r.type = Value::INT;
    r.min_val = static_cast<double>(lo);
    r.max_val = static_cast<double>(hi);
    r.step = static_cast<double>(s);
    return r;
  }

  /// @brief Construct a boolean parameter range (true/false).
  /// @param n Parameter name.
  static ParamRange make_bool(const std::string& n) {
    ParamRange r;
    r.name = n;
    r.type = Value::BOOL;
    r.min_val = 0;
    r.max_val = 1;
    r.step = 1;
    return r;
  }

  /// @brief Number of grid points for this parameter.
  /// @return Number of distinct values in the grid.
  int grid_count() const {
    if (step <= 0 || max_val < min_val) return 1;
    return static_cast<int>(std::floor((max_val - min_val) / step)) + 1;
  }

  /// @brief Convert a grid index to Value.
  /// @param idx Grid index (0-based).
  /// @return The Value at the given index.
  Value at_index(int idx) const {
    double val = min_val + static_cast<double>(idx) * step;
    val = std::min(val, max_val);
    switch (type) {
      case Value::INT:
        return Value(static_cast<int>(std::round(val)));
      case Value::BOOL:
        return Value(val > 0.5);
      default:
        return Value(val);
    }
  }

  /// @brief Sample a random value within this range.
  /// @param gen Random number generator.
  /// @return Random Value within the range.
  template <class RNG>
  Value sample(RNG& gen) const {
    switch (type) {
      case Value::INT: {
        std::uniform_int_distribution<int> dist(static_cast<int>(min_val), static_cast<int>(max_val));
        return Value(dist(gen));
      }
      case Value::BOOL: {
        std::bernoulli_distribution dist(0.5);
        return Value(dist(gen));
      }
      default: {
        std::uniform_real_distribution<double> dist(min_val, max_val);
        return Value(dist(gen));
      }
    }
  }
};

/// @brief Defines the search space as a collection of parameter ranges.
using SearchSpace = std::vector<ParamRange>;

/// @brief A single parameter configuration (one combination of values).
using ParamConfig = ValueManager::NameValueVector;

/// @brief Progress callback type.
///
/// Called after each configuration is evaluated.
/// Arguments: (current_step, total_steps, current_best_score)
/// Return false to cancel the search.
using ProgressCallback = std::function<bool(int, int, double)>;

/// @brief Abstract search strategy interface.
///
/// A search strategy generates a sequence of parameter configurations
/// from a search space definition.
class SearchStrategy {
 public:
  virtual ~SearchStrategy() = default;

  /// @brief Generate all parameter configurations to evaluate.
  /// @param space The search space definition.
  /// @return Vector of parameter configurations.
  virtual std::vector<ParamConfig> generate(const SearchSpace& space) const = 0;

  /// @brief Get the name of this strategy.
  /// @return Strategy name string.
  virtual std::string name() const = 0;
};

/// @brief Exhaustive grid search strategy.
///
/// Generates all combinations of parameter values at the specified
/// grid points (Cartesian product of all parameter ranges).
class GridSearchStrategy : public SearchStrategy {
 public:
  GridSearchStrategy() = default;
  ~GridSearchStrategy() override = default;

  /// @brief Generate all grid combinations.
  /// @param space Search space with parameter ranges.
  /// @return Vector of all grid point configurations.
  std::vector<ParamConfig> generate(const SearchSpace& space) const override {
    std::vector<ParamConfig> configs;
    if (space.empty()) return configs;

    // Total number of combinations
    int total = 1;
    for (const auto& range : space) {
      total *= range.grid_count();
    }
    configs.reserve(static_cast<std::size_t>(total));

    // Generate Cartesian product
    std::vector<int> indices(space.size(), 0);
    for (int i = 0; i < total; ++i) {
      ParamConfig config;
      config.reserve(space.size());
      for (std::size_t p = 0; p < space.size(); ++p) {
        config.emplace_back(space[p].name, space[p].at_index(indices[p]));
      }
      configs.push_back(std::move(config));

      // Increment indices (odometer-style)
      for (int p = static_cast<int>(space.size()) - 1; p >= 0; --p) {
        std::size_t idx = static_cast<std::size_t>(p);
        indices[idx]++;
        if (indices[idx] < space[idx].grid_count()) break;
        indices[idx] = 0;
      }
    }
    return configs;
  }

  std::string name() const override { return "GridSearch"; }
};

/// @brief Random search strategy.
///
/// Generates a specified number of random parameter configurations
/// sampled uniformly from each parameter's range.
class RandomSearchStrategy : public SearchStrategy {
 public:
  /// @brief Construct with sample count and optional seed.
  /// @param n_samples Number of random configurations to generate.
  /// @param seed Random seed (0 for non-deterministic).
  explicit RandomSearchStrategy(int n_samples = 100, unsigned int seed = 0) : n_samples_(n_samples), seed_(seed) {}
  ~RandomSearchStrategy() override = default;

  /// @brief Generate random parameter configurations.
  /// @param space Search space with parameter ranges.
  /// @return Vector of random configurations.
  std::vector<ParamConfig> generate(const SearchSpace& space) const override {
    std::vector<ParamConfig> configs;
    if (space.empty()) return configs;

    std::mt19937 gen;
    if (seed_ != 0) {
      gen.seed(seed_);
    } else {
      std::random_device rd;
      gen.seed(rd());
    }

    configs.reserve(static_cast<std::size_t>(n_samples_));
    for (int i = 0; i < n_samples_; ++i) {
      ParamConfig config;
      config.reserve(space.size());
      for (const auto& range : space) {
        config.emplace_back(range.name, range.sample(gen));
      }
      configs.push_back(std::move(config));
    }
    return configs;
  }

  std::string name() const override { return "RandomSearch"; }

  /// @brief Get the number of samples.
  int n_samples() const { return n_samples_; }

  /// @brief Set the number of samples.
  void set_n_samples(int n) { n_samples_ = n; }

 private:
  int n_samples_;      ///< Number of random configurations.
  unsigned int seed_;  ///< Random seed.
};

}  // namespace lsfm
