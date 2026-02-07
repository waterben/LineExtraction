//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file console_app_interface.cpp
/// @brief Console application interface implementation.

#include <utility/console_app_interface.hpp>

#include <exception>
#include <iostream>

namespace lsfm {


int ConsoleAppInterface::run(int argc, char** argv) {
  try {
    defineArgs();
    parseArgs(argc, argv);
    return run();
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unkown error, abort application!" << std::endl;
  }
  return 1;
}

}  // namespace lsfm
