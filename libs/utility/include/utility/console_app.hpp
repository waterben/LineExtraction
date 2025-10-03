#pragma once

#include <utility/console_app_interface.hpp>
#include <utility/options.hpp>

#include <string>

namespace lsfm {

/// @brief Simple console app base class using utility::Options
class ConsoleApp : public ConsoleAppInterface {
 public:
  using ConsoleAppInterface::ConsoleAppInterface;
  ~ConsoleApp() override = default;

 protected:
  /// @brief Define default parse arguments via utility::Options
  void defineArgs() override;

  /// @brief Parse and eval arguments default arguments.
  void parseArgs(int argc, char** argv) final;

  virtual std::string usage() const { return name() + " [options]"; }

  /// Options parser
  utility::Options opts_{};
  bool help_ = false;
  bool version_flag_ = false;

  bool verbose_ = false;
};

}  // namespace lsfm
