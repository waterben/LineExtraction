"""Feature flags for conditional compilation in LineExtraction project.

This provides CMake-like feature detection and conditional compilation.
Features can be enabled/disabled via command line flags.

Example usage:
  bazel build --//bazel:enable_qt5=true //...
  bazel build --//bazel:enable_cuda=false //...
"""

# Feature configuration helpers
def if_qt5_enabled(if_true, if_false = []):
    """Conditionally include dependencies if Qt5 is enabled."""
    return select({
        "//bazel:qt5_enabled": if_true,
        "//conditions:default": if_false,
    })

def if_cuda_enabled(if_true, if_false = []):
    """Conditionally include dependencies if CUDA is enabled."""
    return select({
        "//bazel:cuda_enabled": if_true,
        "//conditions:default": if_false,
    })

def if_opengl_enabled(if_true, if_false = []):
    """Conditionally include dependencies if OpenGL is enabled."""
    return select({
        "//bazel:opengl_enabled": if_true,
        "//conditions:default": if_false,
    })
