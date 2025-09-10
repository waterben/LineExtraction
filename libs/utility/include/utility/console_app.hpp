#pragma once

#include <boost/program_options.hpp>
#include <utility/console_app_interface.hpp>

#include <string>

namespace lsfm {

/// @brief Simple console app base class using boost::options
class ConsoleApp : public ConsoleAppInterface {
 public:
  using ConsoleAppInterface::ConsoleAppInterface;

 protected:
  /// @brief Define default parse arguments via boost::options
  void defineArgs() override;

  /// @brief Parse and eval arguments default arguments.
  void parseArgs(int argc, char** argv) final;

  /// @brief Check if a specific option exists
  bool isOptionDefined(const std::string& option_name);

  virtual std::string usage() const { return name() + " [options]"; }

  /// Options description
  boost::program_options::options_description options_;
  /// Variables map to map options to member variables
  boost::program_options::variables_map vm_;

  bool verbose_ = false;
};

}  // namespace lsfm
