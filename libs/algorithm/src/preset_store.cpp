//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file preset_store.cpp
/// @brief Implementation of PresetStore — JSON-based preset parameter loading.

#include <algorithm/preset_store.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lsfm {

// ============================================================================
// Construction
// ============================================================================

PresetStore::PresetStore(const std::string& json_path) {
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    throw std::runtime_error("PresetStore: cannot open file: " + json_path);
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  parse(ss.str());
}

PresetStore PresetStore::from_string(const std::string& json_content) {
  PresetStore store;
  store.parse(json_content);
  return store;
}

// ============================================================================
// Lookup
// ============================================================================

const PresetStore::PresetEntry& PresetStore::lookup(const std::string& detector_name,
                                                    const std::string& preset_name) const {
  auto det_it = store_.find(detector_name);
  if (det_it == store_.end()) {
    throw std::out_of_range("PresetStore: no presets for detector '" + detector_name + "'");
  }
  auto pre_it = det_it->second.find(preset_name);
  if (pre_it == det_it->second.end()) {
    throw std::out_of_range("PresetStore: no preset '" + preset_name + "' for detector '" + detector_name + "'");
  }
  return pre_it->second;
}

ParamConfig PresetStore::get(DetectorId detector, const std::string& preset_name) const {
  return lookup(detector_id_to_name(detector), preset_name).params;
}

ParamConfig PresetStore::get(const std::string& detector_name, const std::string& preset_name) const {
  return lookup(detector_name, preset_name).params;
}

double PresetStore::score(DetectorId detector, const std::string& preset_name) const {
  return lookup(detector_id_to_name(detector), preset_name).score;
}

double PresetStore::score(const std::string& detector_name, const std::string& preset_name) const {
  return lookup(detector_name, preset_name).score;
}

bool PresetStore::has(DetectorId detector, const std::string& preset_name) const {
  return has(detector_id_to_name(detector), preset_name);
}

bool PresetStore::has(const std::string& detector_name, const std::string& preset_name) const {
  auto det_it = store_.find(detector_name);
  if (det_it == store_.end()) {
    return false;
  }
  return det_it->second.count(preset_name) > 0;
}

// ============================================================================
// Enumeration
// ============================================================================

std::vector<std::string> PresetStore::preset_names() const {
  std::set<std::string> names;
  for (const auto& [det_name, presets] : store_) {
    for (const auto& [pre_name, entry] : presets) {
      names.insert(pre_name);
    }
  }
  return {names.begin(), names.end()};
}

std::vector<std::string> PresetStore::detector_names() const {
  std::vector<std::string> names;
  names.reserve(store_.size());
  for (const auto& [det_name, presets] : store_) {
    names.push_back(det_name);
  }
  return names;
}

// ============================================================================
// Apply
// ============================================================================

void PresetStore::apply(ValueManager& vm, DetectorId detector, const std::string& preset_name) const {
  vm.value(get(detector, preset_name));
}

void PresetStore::apply(ValueManager& vm, const std::string& detector_name, const std::string& preset_name) const {
  vm.value(get(detector_name, preset_name));
}

// ============================================================================
// JSON parsing
// ============================================================================

void PresetStore::parse(const std::string& json_content) {
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(json_content);
  } catch (const nlohmann::json::parse_error& e) {
    throw std::runtime_error(std::string("PresetStore: JSON parse error: ") + e.what());
  }

  // The "detectors" key contains all detector presets.
  if (!root.contains("detectors") || !root["detectors"].is_object()) {
    throw std::runtime_error("PresetStore: JSON missing 'detectors' object");
  }

  const auto& detectors = root["detectors"];

  for (auto det_it = detectors.begin(); det_it != detectors.end(); ++det_it) {
    const std::string& det_name = det_it.key();
    const auto& det_presets = det_it.value();

    if (!det_presets.is_object()) {
      continue;
    }

    std::map<std::string, PresetEntry> presets;

    for (auto pre_it = det_presets.begin(); pre_it != det_presets.end(); ++pre_it) {
      const std::string& pre_name = pre_it.key();
      const auto& pre_data = pre_it.value();

      if (!pre_data.is_object() || !pre_data.contains("params")) {
        continue;
      }

      PresetEntry entry;

      // Parse score
      if (pre_data.contains("score") && pre_data["score"].is_number()) {
        entry.score = pre_data["score"].get<double>();
      }

      // Parse params: object of name -> value
      const auto& params_obj = pre_data["params"];
      if (params_obj.is_object()) {
        for (auto p_it = params_obj.begin(); p_it != params_obj.end(); ++p_it) {
          const std::string& param_name = p_it.key();
          const auto& val = p_it.value();

          Value v;
          if (val.is_boolean()) {
            v = Value(val.get<bool>());
          } else if (val.is_number_integer()) {
            v = Value(static_cast<int64_t>(val.get<int64_t>()));
          } else if (val.is_number_float()) {
            v = Value(val.get<double>());
          } else if (val.is_string()) {
            v = Value(val.get<std::string>());
          } else {
            // Skip unsupported types
            continue;
          }

          entry.params.emplace_back(param_name, v);
        }
      }

      presets[pre_name] = std::move(entry);
    }

    if (!presets.empty()) {
      store_[det_name] = std::move(presets);
    }
  }
}

}  // namespace lsfm
