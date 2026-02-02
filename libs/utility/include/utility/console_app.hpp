/// @file console_app.hpp
/// @brief Console application base class with argument parsing.

#pragma once

#include <utility/console_app_interface.hpp>
#include <utility/options.hpp>

#include <string>

namespace lsfm {

/// @brief Console application base with integrated Options parsing.
///
/// Extends ConsoleAppInterface with automatic argument parsing via
/// utility::Options. Provides built-in --help, --version, and --verbose flags.
class ConsoleApp : public ConsoleAppInterface {
 public:
  using ConsoleAppInterface::ConsoleAppInterface;
  ~ConsoleApp() override = default;

 protected:
  /// @brief Define default arguments (help, version, verbose).
  void defineArgs() override;

  /// @brief Parse command-line arguments.
  /// @param argc Argument count.
  /// @param argv Argument values.
  void parseArgs(int argc, char** argv) final;

  /// @brief Get usage string for help message.
  /// @return Usage string.
  virtual std::string usage() const { return name() + " [options]"; }

  utility::Options opts_{};    ///< Options parser instance.
  bool help_ = false;          ///< Help flag was set.
  bool version_flag_ = false;  ///< Version flag was set.
  bool verbose_ = false;       ///< Verbose output enabled.
};

}  // namespace lsfm
