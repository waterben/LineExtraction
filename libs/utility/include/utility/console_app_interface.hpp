#pragma once

#include <string>
#include <utility>

namespace lsfm {

/// @brief Simple console app base interface
class ConsoleAppInterface {
 public:
  explicit ConsoleAppInterface(std::string name, std::string description = "", std::string version = "1.0.0")
      : name_(std::move(name)), description_(std::move(description)), version_(std::move(version)) {}

  /// @brief Main entry point for application
  /// @param argc main argc
  /// @param argv main argv
  /// @return exit code
  int run(int argc, char** argv);

  /// @brief Return application name
  const std::string& name() const { return name_; }
  /// @brief Return application description
  const std::string& description() const { return description_; }
  /// @brief Return application version
  const std::string& version() const { return version_; }

 protected:
  /// @brief Define parse arguments
  virtual void defineArgs() = 0;

  /// @brief Parse and eval arguments. If invalid, should print help and exit
  virtual void parseArgs(int argc, char** argv) = 0;

  /// @brief Run application
  virtual int run() = 0;

 private:
  std::string name_{};
  std::string description_{};
  std::string version_{"1.0.0"};
};

}  // namespace lsfm
