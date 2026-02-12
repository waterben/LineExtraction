"""Macros for building pybind11 Python extension modules in LineExtraction."""

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_python//python:defs.bzl", "py_library")
load("//tools/bazel:stubgen.bzl", "le_pybind_stubgen")

def _python_tag_select():
    """Select the Python ABI tag based on the configured Python version."""
    return select({
        "@rules_python//python/config_settings:is_python_3.8": "cp38",
        "@rules_python//python/config_settings:is_python_3.9": "cp39",
        "@rules_python//python/config_settings:is_python_3.10": "cp310",
        "@rules_python//python/config_settings:is_python_3.11": "cp311",
        "@rules_python//python/config_settings:is_python_3.12": "cp312",
        "//conditions:default": "cp311",
    })

def le_pybind_module(
        name,
        srcs,
        deps,
        py_deps = [],
        py_target_name = None,
        generate_stubs = False,
        stub_deps = [],
        visibility = None,
        **kwargs):
    """Create a pybind11 extension module with a corresponding py_library target.

    This macro creates:
      1. A pybind_extension (shared library loadable by Python)
      2. A py_library wrapping the extension for use as a Bazel dependency
      3. (Optional) A genrule that generates a .pyi type stub at build time

    Args:
        name: The name of the pybind11 module (used as the Python import name).
        srcs: C++ source files for the pybind11 module.
        deps: C++ dependencies for the pybind11 module.
        py_deps: Python dependencies for the py_library target.
        py_target_name: Name of the py_library target. Defaults to name + "_lib".
        generate_stubs: If True, generate .pyi type stubs at build time using
            pybind11-stubgen. The stub is included in the py_library data.
        stub_deps: Other pybind_extension .so targets that this module imports
            at load time (needed so stubgen can resolve cross-module references).
            Only relevant when generate_stubs = True.
        visibility: Bazel visibility for the targets.
        **kwargs: Additional keyword arguments passed to pybind_extension.
    """

    # Export README.md for documentation if it exists
    readme_files = native.glob(["README.md"], allow_empty = True)
    if readme_files:
        native.exports_files(["README.md"])

    pybind_extension(
        name = name,
        srcs = srcs,
        deps = deps,
        visibility = visibility,
        **kwargs
    )

    # Build data list: always include the .so extension
    lib_data = [":" + name]

    # Generate type stub (.pyi) at build time if requested
    if generate_stubs:
        le_pybind_stubgen(
            name = name,
            extension = ":" + name + ".so",
            stub_deps = stub_deps,
            visibility = visibility,
        )
        lib_data.append(":" + name + "_pyi")

    library_name = name + "_lib" if (py_target_name == None) else py_target_name
    py_library(
        name = library_name,
        data = lib_data,
        deps = py_deps,
        imports = ["."],
        visibility = visibility if visibility else ["//visibility:public"],
    )
