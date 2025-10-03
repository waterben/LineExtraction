/// @brief Minimal command line options parser to replace boost::program_options for common use cases.
///
/// Features:
///  - Long options: --input value or --input=value
///  - Short options: -i value (single-letter)
///  - Boolean switches: --flag or -f set true
///  - Required and default values for string options
///  - Remaining (positional) arguments are returned from parse
///
/// Limitations:
///  - No multi-token options, no repeated options aggregation
///  - No typed parsing beyond std::string and bool
///  - No automatic help generation (but descriptions are stored)
#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace utility {

/// @brief Exception thrown on parsing errors.
class options_error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// @brief Minimal options parser for strings and boolean switches.
class Options {
 public:
  /// @brief Register a string option.
  /// @param long_name Long option name without leading dashes (e.g. "input").
  /// @param short_name Optional short name (single character) or '\0' when unused.
  /// @param description Human-readable description.
  /// @param target Reference to a std::string to store the parsed value.
  /// @param required If true, option must be provided by the user.
  /// @param default_value Optional default value applied if not provided (and not required).
  void add_string(const std::string& long_name,
                  char short_name,
                  const std::string& description,
                  std::string& target,
                  bool required = false,
                  const std::string& default_value = std::string());

  /// @brief Register a boolean switch.
  /// @param long_name Long option name without leading dashes.
  /// @param short_name Optional short name (single character) or '\0'.
  /// @param description Human-readable description.
  /// @param target Reference to a bool to set true when the switch appears.
  void add_switch(const std::string& long_name, char short_name, const std::string& description, bool& target);

  /// @brief Parse argv for registered options.
  /// @param argc Argument count.
  /// @param argv Argument vector.
  /// @return Vector of remaining positional arguments.
  /// @throws options_error on invalid syntax, unknown options, or missing required values.
  std::vector<std::string> parse(int argc, char** argv) const;

 private:
  struct EntryBase {
    std::string long_name{};
    char short_name{};
    std::string description{};
    mutable bool seen{false};

    virtual ~EntryBase() = default;

   protected:
    EntryBase(std::string long_n, char short_n, std::string desc)
        : long_name(std::move(long_n)), short_name(short_n), description(std::move(desc)) {}

   public:
    EntryBase() = default;
    virtual void set_from_string(const std::string& value) const = 0;
    virtual void set_switch() const = 0;
    virtual bool is_required() const = 0;
    virtual bool has_default() const = 0;
    virtual void apply_default() const = 0;
    virtual bool expects_value() const = 0;
    bool was_set() const { return seen; }
  };

  struct StringEntry : EntryBase {
    std::string& target;
    bool required{false};
    bool has_def{false};
    std::string def_value;
    StringEntry(std::string long_n, char short_n, std::string desc, std::string& tgt, bool req, std::string def)
        : EntryBase(std::move(long_n), short_n, std::move(desc)),
          target(tgt),
          required(req),
          has_def(!def.empty()),
          def_value(std::move(def)) {}
    void set_from_string(const std::string& value) const override {
      target = value;
      seen = true;
    }
    void set_switch() const override { throw options_error("Option --" + long_name + " expects a value"); }
    bool is_required() const override { return required; }
    bool has_default() const override { return has_def; }
    void apply_default() const override {
      if (has_def) target = def_value;
    }
    bool expects_value() const override { return true; }
  };

  struct SwitchEntry : EntryBase {
    bool& target;
    SwitchEntry(std::string long_n, char short_n, std::string desc, bool& tgt)
        : EntryBase(std::move(long_n), short_n, std::move(desc)), target(tgt) {}
    void set_from_string(const std::string&) const override {
      target = true;
      seen = true;
    }
    void set_switch() const override {
      target = true;
      seen = true;
    }
    bool is_required() const override { return false; }
    bool has_default() const override { return false; }
    void apply_default() const override {}
    bool expects_value() const override { return false; }
  };

  std::vector<std::unique_ptr<EntryBase>> entries_{};
  std::unordered_map<std::string, EntryBase*> by_long_{};
  std::unordered_map<char, EntryBase*> by_short_{};
};

}  // namespace utility
