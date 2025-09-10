#include <utility/console_app.hpp>

#include <fstream>
#include <iostream>

namespace po = boost::program_options;

namespace lsfm {

void ConsoleApp::defineArgs() {
  // clang-format off
  options_.add_options()
  ("help,h", "Show help message")
  ("version,v", "Show application version")
  ("verbose", po::bool_switch(&verbose_), "Enable verbose output")
  ("config,c", po::value<std::string>()->default_value("config.ini"), "Configuration file");
  // clang-format on
}

void ConsoleApp::parseArgs(int argc, char** argv) {
  // First pass: Only parse for --config option if it's defined
  if (isOptionDefined("config")) {
    po::options_description config_options("Config Options");
    config_options.add_options()("config,c", po::value<std::string>(), "Configuration file");

    po::variables_map temp_vm;
    po::store(
        boost::program_options::command_line_parser(argc, argv).options(config_options).allow_unregistered().run(),
        temp_vm);

    if (temp_vm.count("config")) {
      // Parse config file if it exists
      std::ifstream ifs(temp_vm["config"].as<std::string>());
      if (ifs) {
        po::store(po::parse_config_file(ifs, options_), vm_);
      }
    }
  }

  // Second pass: Parse full command-line arguments (command-line overrides config file)
  po::store(po::parse_command_line(argc, argv, options_), vm_);

  // Now enforce required options after merging command-line and config file values
  try {
    po::notify(vm_);
  } catch (const po::required_option& e) {
    std::cerr << "Error: " << e.what() << "\n\n";
    std::cerr << options_ << std::endl;
    std::exit(1);
  } catch (const po::error& e) {
    std::cerr << "Error: " << e.what() << "\n\n";
    std::cerr << options_ << std::endl;
    std::exit(1);
  }

  if (vm_.count("help")) {
    std::cout << "Usage: " << usage() << std::endl;
    std::cout << "Allowed option:" << std::endl;
    std::cout << options_ << std::endl;
    std::exit(0);
  }

  if (vm_.count("version")) {
    std::cout << name() << " version " << version() << std::endl;
    std::exit(0);
  }
}

bool ConsoleApp::isOptionDefined(const std::string& option_name) {
  for (const auto& opt : options_.options()) {
    if (opt->long_name() == option_name) {
      return true;
    }
  }
  return false;
}

}  // namespace lsfm
