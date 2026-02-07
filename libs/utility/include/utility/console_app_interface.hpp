//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file console_app_interface.hpp
/// @brief Abstract base interface for console applications.

#pragma once

#include <string>
#include <utility>

namespace lsfm {

/// @brief Abstract interface for console applications.
///
/// Provides structure for command-line applications with argument parsing
/// and application lifecycle management.
class ConsoleAppInterface {
 public:
  /// @brief Construct with application metadata.
  /// @param name Application name.
  /// @param description Short description.
  /// @param version Version string.
  explicit ConsoleAppInterface(std::string name, std::string description = "", std::string version = "1.0.0")
      : name_(std::move(name)), description_(std::move(description)), version_(std::move(version)) {}

  virtual ~ConsoleAppInterface() = default;

  /// @brief Main entry point.
  /// @param argc Argument count from main().
  /// @param argv Argument values from main().
  /// @return Exit code.
  int run(int argc, char** argv);

  /// @brief Get application name.
  const std::string& name() const { return name_; }

  /// @brief Get application description.
  const std::string& description() const { return description_; }

  /// @brief Get application version.
  const std::string& version() const { return version_; }

 protected:
  /// @brief Define command-line arguments.
  virtual void defineArgs() = 0;

  /// @brief Parse command-line arguments.
  /// @param argc Argument count.
  /// @param argv Argument values.
  virtual void parseArgs(int argc, char** argv) = 0;

  /// @brief Execute application logic.
  /// @return Exit code.
  virtual int run() = 0;

 private:
  std::string name_{};            ///< Application name.
  std::string description_{};     ///< Application description.
  std::string version_{"1.0.0"};  ///< Application version.
};

}  // namespace lsfm
