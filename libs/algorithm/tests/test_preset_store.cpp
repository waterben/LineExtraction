//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************

#include <algorithm/preset_store.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace lsfm;

// =============================================================================
// Test JSON data
// =============================================================================

/// @brief Minimal valid presets JSON for testing.
static const std::string kTestJson = R"({
  "metadata": {
    "num_images": 10,
    "num_samples": 50
  },
  "detectors": {
    "LsdCC": {
      "fast": {
        "params": {
          "nms_th_low": 0.01,
          "nms_th_high": 0.03,
          "edge_min_pixels": 5,
          "edge_max_gap": 2,
          "split_error_distance": 1.5
        },
        "score": 0.85
      },
      "balanced": {
        "params": {
          "nms_th_low": 0.008,
          "nms_th_high": 0.025,
          "edge_min_pixels": 8,
          "edge_max_gap": 1,
          "split_error_distance": 2.0
        },
        "score": 0.90
      },
      "accurate": {
        "params": {
          "nms_th_low": 0.004,
          "nms_th_high": 0.015,
          "edge_min_pixels": 3,
          "edge_max_gap": 3,
          "split_error_distance": 3.0
        },
        "score": 0.78
      }
    },
    "LsdFGioi": {
      "fast": {
        "params": {
          "quant_error": 2.0,
          "angle_th": 22.5,
          "log_eps": 0.0,
          "density_th": 0.7,
          "bins": 1024
        },
        "score": 0.82
      },
      "balanced": {
        "params": {
          "quant_error": 1.5,
          "angle_th": 25.0,
          "log_eps": -1.0,
          "density_th": 0.5,
          "bins": 1024
        },
        "score": 0.88
      }
    }
  }
})";

// =============================================================================
// Construction tests
// =============================================================================

TEST(PresetStoreTest, DefaultConstruction) {
  PresetStore store;
  EXPECT_TRUE(store.empty());
  EXPECT_EQ(store.num_detectors(), 0u);
}

TEST(PresetStoreTest, FromString) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_FALSE(store.empty());
  EXPECT_EQ(store.num_detectors(), 2u);
}

TEST(PresetStoreTest, InvalidJsonThrows) {
  EXPECT_THROW(PresetStore::from_string("{invalid json"), std::runtime_error);
}

TEST(PresetStoreTest, MissingDetectorsKeyThrows) {
  EXPECT_THROW(PresetStore::from_string(R"({"metadata": {}})"), std::runtime_error);
}

TEST(PresetStoreTest, NonexistentFileThrows) {
  EXPECT_THROW(PresetStore("/nonexistent/path/presets.json"), std::runtime_error);
}

// =============================================================================
// Enumeration tests
// =============================================================================

TEST(PresetStoreTest, DetectorNames) {
  auto store = PresetStore::from_string(kTestJson);
  auto names = store.detector_names();
  EXPECT_EQ(names.size(), 2u);
  // std::map iteration is sorted
  EXPECT_EQ(names[0], "LsdCC");
  EXPECT_EQ(names[1], "LsdFGioi");
}

TEST(PresetStoreTest, PresetNames) {
  auto store = PresetStore::from_string(kTestJson);
  auto names = store.preset_names();
  // Union of all preset names across detectors, sorted
  EXPECT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "accurate");
  EXPECT_EQ(names[1], "balanced");
  EXPECT_EQ(names[2], "fast");
}

// =============================================================================
// Lookup tests (by DetectorId)
// =============================================================================

TEST(PresetStoreTest, GetByDetectorId) {
  auto store = PresetStore::from_string(kTestJson);
  auto params = store.get(DetectorId::LSD_CC, "balanced");
  EXPECT_FALSE(params.empty());

  // Check that expected parameter names are present
  bool found_nms_low = false;
  bool found_edge_pixels = false;
  for (const auto& nv : params) {
    if (nv.name == "nms_th_low") {
      EXPECT_NEAR(nv.value.getDouble(), 0.008, 1e-6);
      found_nms_low = true;
    }
    if (nv.name == "edge_min_pixels") {
      EXPECT_EQ(nv.value.getInt(), 8);
      found_edge_pixels = true;
    }
  }
  EXPECT_TRUE(found_nms_low);
  EXPECT_TRUE(found_edge_pixels);
}

TEST(PresetStoreTest, GetByDetectorIdFast) {
  auto store = PresetStore::from_string(kTestJson);
  auto params = store.get(DetectorId::LSD_CC, "fast");
  EXPECT_EQ(params.size(), 5u);
}

TEST(PresetStoreTest, GetByDetectorIdUnknownPresetThrows) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_THROW(store.get(DetectorId::LSD_CC, "nonexistent"), std::out_of_range);
}

TEST(PresetStoreTest, GetByDetectorIdUnknownDetectorThrows) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_THROW(store.get(DetectorId::LSD_BURNS, "balanced"), std::out_of_range);
}

// =============================================================================
// Lookup tests (by name string)
// =============================================================================

TEST(PresetStoreTest, GetByName) {
  auto store = PresetStore::from_string(kTestJson);
  auto params = store.get("LsdFGioi", "fast");
  EXPECT_EQ(params.size(), 5u);

  bool found_bins = false;
  for (const auto& nv : params) {
    if (nv.name == "bins") {
      EXPECT_EQ(nv.value.getInt(), 1024);
      found_bins = true;
    }
  }
  EXPECT_TRUE(found_bins);
}

TEST(PresetStoreTest, GetByNameUnknownThrows) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_THROW(store.get("NoSuchDetector", "balanced"), std::out_of_range);
}

// =============================================================================
// Score tests
// =============================================================================

TEST(PresetStoreTest, ScoreByDetectorId) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_NEAR(store.score(DetectorId::LSD_CC, "fast"), 0.85, 1e-6);
  EXPECT_NEAR(store.score(DetectorId::LSD_CC, "balanced"), 0.90, 1e-6);
  EXPECT_NEAR(store.score(DetectorId::LSD_CC, "accurate"), 0.78, 1e-6);
}

TEST(PresetStoreTest, ScoreByName) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_NEAR(store.score("LsdFGioi", "balanced"), 0.88, 1e-6);
}

TEST(PresetStoreTest, ScoreUnknownThrows) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_THROW(store.score(DetectorId::LSD_CC, "nonexistent"), std::out_of_range);
}

// =============================================================================
// has() tests
// =============================================================================

TEST(PresetStoreTest, Has) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_TRUE(store.has(DetectorId::LSD_CC, "fast"));
  EXPECT_TRUE(store.has(DetectorId::LSD_CC, "balanced"));
  EXPECT_TRUE(store.has(DetectorId::LSD_CC, "accurate"));
  EXPECT_FALSE(store.has(DetectorId::LSD_CC, "nonexistent"));
  EXPECT_FALSE(store.has(DetectorId::LSD_BURNS, "fast"));
}

TEST(PresetStoreTest, HasByName) {
  auto store = PresetStore::from_string(kTestJson);
  EXPECT_TRUE(store.has("LsdFGioi", "fast"));
  EXPECT_TRUE(store.has("LsdFGioi", "balanced"));
  EXPECT_FALSE(store.has("LsdFGioi", "accurate"));
  EXPECT_FALSE(store.has("NoSuch", "fast"));
}

// =============================================================================
// Constants
// =============================================================================

TEST(PresetStoreTest, Constants) {
  EXPECT_STREQ(PresetStore::FAST, "fast");
  EXPECT_STREQ(PresetStore::BALANCED, "balanced");
  EXPECT_STREQ(PresetStore::ACCURATE, "accurate");
}

// =============================================================================
// Edge cases
// =============================================================================

TEST(PresetStoreTest, EmptyDetectorsObject) {
  auto store = PresetStore::from_string(R"({"metadata": {}, "detectors": {}})");
  EXPECT_TRUE(store.empty());
  EXPECT_EQ(store.num_detectors(), 0u);
  EXPECT_TRUE(store.preset_names().empty());
  EXPECT_TRUE(store.detector_names().empty());
}

TEST(PresetStoreTest, PresetWithMissingParams) {
  // Preset entry without "params" key should be skipped
  auto store = PresetStore::from_string(R"({
    "detectors": {
      "LsdCC": {
        "fast": { "score": 0.5 },
        "balanced": { "params": {"x": 1.0}, "score": 0.9 }
      }
    }
  })");
  EXPECT_FALSE(store.has(DetectorId::LSD_CC, "fast"));
  EXPECT_TRUE(store.has(DetectorId::LSD_CC, "balanced"));
}

TEST(PresetStoreTest, BooleanParams) {
  auto store = PresetStore::from_string(R"({
    "detectors": {
      "LsdEDLZ": {
        "fast": {
          "params": { "validate": true, "grad_th": 10.0 },
          "score": 0.75
        }
      }
    }
  })");
  auto params = store.get(DetectorId::LSD_EDLZ, "fast");
  EXPECT_EQ(params.size(), 2u);

  bool found_bool = false;
  for (const auto& nv : params) {
    if (nv.name == "validate") {
      EXPECT_TRUE(nv.value.getBool());
      found_bool = true;
    }
  }
  EXPECT_TRUE(found_bool);
}
