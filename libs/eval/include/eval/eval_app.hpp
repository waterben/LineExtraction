//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file eval_app.hpp
/// @brief Evaluation application framework and runner.
/// Provides high-level application interface for running evaluation tasks.

#pragma once

#include <utility/console_app.hpp>

namespace lsfm {

/// @brief Evaluation app interface which provides basic set of options
class EvalApp : public ConsoleApp {
 public:
  using ConsoleApp::ConsoleApp;

 protected:
  /// @brief Get args for eval
  void defineArgs() override;

  /// @brief Run application
  int run() final;

  /// @brief Initialize evaluation
  virtual void initEval() {};

  /// @brief Run evaluation
  virtual void runEval() = 0;

  /// @brief Terminate evaluation
  virtual void terminateEval() {};

  // Flags and options
  std::string input_{};        ///< Input path (file or directory)
  std::string output_{};       ///< Output path for results
  bool recursive_{false};      ///< Scan input directories recursively
  bool run_high_prio_{false};  ///< Run with elevated process priority
  bool no_results_{false};     ///< Suppress result output
  bool write_visuals_{false};  ///< Write visual results to disk
  bool show_visuals_{false};   ///< Display visual results interactively
};

}  // namespace lsfm
