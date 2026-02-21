---
applyTo: "**/*.{cpp,hpp,c,cc,cxx,h,hxx}"
---

Write Doxygen-style documentation for C++ code following Google C++ Style Guide and C++ Core Guidelines. Here are some examples:

**Function Documentation:**

```cpp

/// @brief Calculate the factorial of a number.
/// This function calculates the factorial of a given non-negative integer
/// using iterative approach for better performance.
/// @param n The number to calculate factorial for (must be >= 0)
/// @return The factorial of n, or 1 if n is 0
/// @throws std::invalid_argument if n is negative
/// @example
/// @code{cpp}
/// int result = factorial(5);  // Returns 120
/// @endcode
int factorial(int n);
```

**Class Documentation:**

```cpp

/// @brief A simple container class for managing integers.
/// This class provides basic operations for storing and manipulating
/// a collection of integers with automatic memory management.
class IntegerContainer {
private:
    /// @brief Internal storage for integers.
    std::vector<int> data_{};

public:
    IntegerContainer();

    /// @brief Add an integer to the container.
    /// @param value The integer value to add
    void add(int value);

    /// @brief Get the size of the container.
    /// @return The number of elements in the container
    size_t size() const;
};
```

**Template Documentation:**

```cpp

/// @brief Generic container template for any type.
/// @tparam T The type of elements to store
template<typename T>
class Container {

    /// @brief Add an element to the container.
    /// @tparam U Type that can be converted to T
    /// @param value The value to add
    template<typename U>
    void add(U&& value);
};
```

**Google C++ Style Guide Requirements:**

- Use snake_case for function and variable names
- Use PascalCase for class and struct names
- Prefer `const` references for parameters that won't be modified
- Use `explicit` for single-argument constructors
- Avoid using namespace declarations in headers
- Prefer `#include <system>` before `#include "local"`
- Use 2-space indentation
- Keep line length under 100 characters when practical

**C++ Core Guidelines Compliance:**

- Follow RAII (Resource Acquisition Is Initialization)
- Prefer smart pointers over raw pointers for ownership
- Use `const` whenever possible
- Prefer range-based for loops
- Use `auto` when type is obvious from context
- Avoid C-style casts, prefer static_cast, dynamic_cast, etc.
- Use `nullptr` instead of `NULL` or `0`
- Prefer standard library algorithms over hand-written loops
- Use move semantics for expensive-to-copy objects
- Make interfaces explicit and type-safe

**Initialization and Resource Management (CRITICAL):**

- **Always initialize ALL member variables** in the member initialization list
- Use default member initializers (`Type member_{};` or `Type member_ = value;`) for simple defaults
- Initialize members in declaration order in the constructor initialization list
- This is enforced by `-Werror=effc++` in both CMake and Bazel builds

```cpp
// CORRECT: All members initialized
class GoodExample {
  cv::Mat data_{};           // Default member initializer
  int count_ = 0;            // Explicit default value
  std::string name_{};       // Default initialization

 public:
  GoodExample() : data_(), count_(0), name_() {}  // Explicit in init list
  GoodExample(int n) : data_(), count_(n), name_() {}  // Even when not all used
};

// WRONG: Missing member initializers - will cause -Werror=effc++ failure
class BadExample {
  cv::Mat data_;    // No default initializer
  int count_;       // Uninitialized!

 public:
  BadExample() {}   // ERROR: members not in initialization list
};
```

**Language:**

- Use **American English** spelling in all comments, Doxygen docs, and identifiers.
  - `color` not `colour`, `gray` not `grey`, `initialize` not `initialise`, `optimize` not `optimise`,
    `normalize` not `normalise`, `neighbor` not `neighbour`, `behavior` not `behaviour`, etc.

**Additional Documentation Guidelines:**

- Use `@brief` for short descriptions
- Use `@param` for parameters with clear descriptions
- Use `@return` or `@returns` for return value descriptions
- Use `@throws` or `@exception` for exceptions that may be thrown
- Use `@see` for cross-references
- Use `@note`, `@warning`, `@attention` for important information
- Use `@example` to provide usage examples
- Use `@deprecated` for deprecated functions/classes
- Use `@since` to indicate version when feature was added
- Use `@author` and `@version` for class-level documentation
- Document all public methods, classes, and important private members
- Keep descriptions concise but informative
- Use proper grammar and complete sentences
