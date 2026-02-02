/// @file runfiles.hpp
/// @brief Bazel Runfiles helper for locating data files at runtime.
///
/// This header provides utilities for resolving paths to data files that are
/// declared as Bazel data dependencies. It supports both Bazel builds (using
/// the runfiles library) and non-Bazel builds (using relative paths as fallback).
///
/// @example
/// @code
/// // In your test or binary:
/// auto runfiles = lsfm::Runfiles::Create(argv[0]);
/// std::string path = runfiles->Rlocation("line_extraction/resources/datasets/BSDS500");
/// @endcode
#pragma once

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

// Bazel runfiles library (only available in Bazel builds)
#if __has_include("tools/cpp/runfiles/runfiles.h")
#  include "tools/cpp/runfiles/runfiles.h"
#  define LSFM_HAS_BAZEL_RUNFILES 1
#else
#  define LSFM_HAS_BAZEL_RUNFILES 0
#endif

namespace lsfm {

/// @brief Helper class for locating Bazel runfiles or falling back to relative paths.
///
/// This class provides a unified interface for locating data files regardless of
/// whether the code is running under Bazel or as a standalone executable.
class Runfiles {
 public:
  virtual ~Runfiles() = default;

  /// @brief Resolve a runfile path to an absolute filesystem path.
  /// @param path The runfile path (e.g., "line_extraction/resources/datasets/BSDS500")
  /// @return The resolved absolute filesystem path
  virtual std::string Rlocation(const std::string& path) const = 0;

  /// @brief Check if running under Bazel (with runfiles support)
  virtual bool isBazelRun() const = 0;

  /// @brief Create a Runfiles instance.
  /// @param argv0 The argv[0] from main() - used by Bazel to locate runfiles
  /// @param fallback_base Optional base path for non-Bazel fallback (defaults to current dir)
  /// @return A Runfiles instance
  static std::unique_ptr<Runfiles> Create(const std::string& argv0, const std::filesystem::path& fallback_base = {});

  /// @brief Create a Runfiles instance for tests (without argv[0]).
  /// @param fallback_base Optional base path for non-Bazel fallback
  /// @return A Runfiles instance
  static std::unique_ptr<Runfiles> CreateForTest(const std::filesystem::path& fallback_base = {});
};

#if LSFM_HAS_BAZEL_RUNFILES

/// @brief Bazel Runfiles implementation using the official runfiles library.
class BazelRunfiles : public Runfiles {
 public:
  explicit BazelRunfiles(std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles)
      : runfiles_(std::move(runfiles)) {}

  std::string Rlocation(const std::string& path) const override { return runfiles_->Rlocation(path); }

  bool isBazelRun() const override { return true; }

 private:
  std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles_;
};

#endif  // LSFM_HAS_BAZEL_RUNFILES

/// @brief Fallback implementation for non-Bazel builds using relative paths.
class FallbackRunfiles : public Runfiles {
 public:
  explicit FallbackRunfiles(std::filesystem::path base_path) : base_path_(std::move(base_path)) {}

  std::string Rlocation(const std::string& path) const override {
    // Strip the workspace name prefix if present (e.g., "line_extraction/")
    std::string stripped_path = path;
    const std::string workspace_prefix = "line_extraction/";
    if (path.find(workspace_prefix) == 0) {
      stripped_path = path.substr(workspace_prefix.length());
    }
    return (base_path_ / stripped_path).string();
  }

  bool isBazelRun() const override { return false; }

 private:
  std::filesystem::path base_path_;
};

inline std::unique_ptr<Runfiles> Runfiles::Create(const std::string& argv0,
                                                  const std::filesystem::path& fallback_base) {
#if LSFM_HAS_BAZEL_RUNFILES
  std::string error;
  auto runfiles = bazel::tools::cpp::runfiles::Runfiles::Create(argv0, &error);
  if (runfiles) {
    return std::make_unique<BazelRunfiles>(std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles>(runfiles));
  }
  // Bazel runfiles not available, fall through to fallback
#endif

  // Fallback: use relative paths from executable directory or provided base
  std::filesystem::path base = fallback_base;
  if (base.empty()) {
    // Try to determine base from argv0 or current directory
    std::filesystem::path exe_path(argv0);
    if (exe_path.has_parent_path() && std::filesystem::exists(exe_path.parent_path())) {
      base = exe_path.parent_path();
    } else {
      base = std::filesystem::current_path();
    }
  }
  return std::make_unique<FallbackRunfiles>(base);
}

inline std::unique_ptr<Runfiles> Runfiles::CreateForTest(const std::filesystem::path& fallback_base) {
#if LSFM_HAS_BAZEL_RUNFILES
  std::string error;
  auto runfiles = bazel::tools::cpp::runfiles::Runfiles::CreateForTest(&error);
  if (runfiles) {
    return std::make_unique<BazelRunfiles>(std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles>(runfiles));
  }
#endif

  std::filesystem::path base = fallback_base;
  if (base.empty()) {
    // Check for TEST_SRCDIR environment variable (set by Bazel)
    const char* test_srcdir = std::getenv("TEST_SRCDIR");
    if (test_srcdir) {
      base = test_srcdir;
    } else {
      base = std::filesystem::current_path();
    }
  }
  return std::make_unique<FallbackRunfiles>(base);
}

}  // namespace lsfm
