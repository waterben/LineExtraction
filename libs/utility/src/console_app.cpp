//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file console_app.cpp
/// @brief Console application implementation.

#include <utility/console_app.hpp>

#include <fstream>
#include <iostream>

namespace lsfm {

void ConsoleApp::defineArgs() {
  opts_.add_switch("help", 'h', "Show help message", help_);
  opts_.add_switch("version", 'v', "Show application version", version_flag_);
  opts_.add_switch("verbose", '\0', "Enable verbose output", verbose_);
}

void ConsoleApp::parseArgs(int argc, char** argv) {
  try {
    auto rest = opts_.parse(argc, argv);
    (void)rest;  // unused positionals for now
  } catch (const utility::options_error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::exit(1);
  }

  if (help_) {
    std::cout << "Usage: " << usage() << std::endl;
    std::exit(0);
  }
  if (version_flag_) {
    std::cout << name() << " version " << version() << std::endl;
    std::exit(0);
  }
}

}  // namespace lsfm
