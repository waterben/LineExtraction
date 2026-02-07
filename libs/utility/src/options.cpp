//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file options.cpp
/// @brief Options utilities implementation.

#include <utility/options.hpp>

#include <algorithm>
#include <memory>

namespace utility {

void Options::add_string(const std::string& long_name,
                         char short_name,
                         const std::string& description,
                         std::string& target,
                         bool required,
                         const std::string& default_value) {
  auto e = std::make_unique<StringEntry>(long_name, short_name, description, target, required, default_value);
  auto p = e.get();
  entries_.push_back(std::move(e));
  by_long_[p->long_name] = p;
  if (short_name) by_short_[short_name] = p;
}

void Options::add_switch(const std::string& long_name, char short_name, const std::string& description, bool& target) {
  auto e = std::make_unique<SwitchEntry>(long_name, short_name, description, target);
  auto p = e.get();
  entries_.push_back(std::move(e));
  by_long_[p->long_name] = p;
  if (short_name) by_short_[short_name] = p;
}

std::vector<std::string> Options::parse(int argc, char** argv) const {
  // Apply defaults
  for (const auto& e : entries_) e->apply_default();

  std::vector<std::string> positionals;

  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.rfind("--", 0) == 0) {
      // Long option
      std::string name, value;
      auto eq = arg.find('=');
      if (eq == std::string::npos) {
        name = arg.substr(2);
      } else {
        name = arg.substr(2, eq - 2);
        value = arg.substr(eq + 1);
      }
      auto it = by_long_.find(name);
      if (it == by_long_.end()) throw options_error("Unknown option --" + name);
      auto* e = it->second;
      if (value.empty()) {
        // switch or value separated by space
        if (e->expects_value()) {
          if (i + 1 < argc && argv[i + 1][0] != '-') {
            e->set_from_string(argv[++i]);
          } else {
            throw options_error("Option --" + name + " expects a value");
          }
        } else {
          e->set_switch();
        }
      } else {
        e->set_from_string(value);
      }
    } else if (arg.rfind('-', 0) == 0 && arg.size() >= 2) {
      // Short option
      char c = arg[1];
      auto it = by_short_.find(c);
      if (it == by_short_.end()) throw options_error(std::string("Unknown option -") + c);
      auto* e = it->second;
      if (arg.size() > 2) {
        // Support -iValue syntax
        e->set_from_string(arg.substr(2));
      } else {
        if (e->expects_value()) {
          if (i + 1 < argc && argv[i + 1][0] != '-') {
            e->set_from_string(argv[++i]);
          } else {
            throw options_error(std::string("Option -") + c + " expects a value");
          }
        } else {
          e->set_switch();
        }
      }
    } else {
      positionals.push_back(arg);
    }
  }

  // Check required
  for (const auto& e : entries_) {
    if (e->is_required() && !e->was_set()) {
      throw options_error("Missing required option --" + e->long_name);
    }
  }

  return positionals;
}

}  // namespace utility
