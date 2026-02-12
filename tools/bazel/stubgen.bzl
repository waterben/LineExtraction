"""Bazel macro: generate .pyi type stubs from built pybind11 extensions.

Uses ``pybind11-stubgen`` (via ``@pip//pybind11_stubgen``) to introspect a
compiled ``.so`` module and produce a ``.pyi`` stub file.  The macro wraps a
``genrule`` that runs entirely inside Bazel's action graph, so stubs are
always in sync with the C++ code — no out-of-band scripts needed.

Typical usage (via ``le_pybind_module`` in ``pybind.bzl``)::

    le_pybind_module(
        name = "le_imgproc",
        srcs = [...],
        deps = [...],
        generate_stubs = True,   # ← generates le_imgproc.pyi automatically
    )
"""

def le_pybind_stubgen(
        name,
        extension,
        stub_deps = [],
        visibility = None):
    """Generate a .pyi type stub from a pybind11 .so extension module.

    Creates a genrule that:
      1. Locates the compiled .so and any dependency .so files
      2. Runs pybind11-stubgen via the ``//tools/bazel:run_stubgen`` tool
      3. Outputs ``<name>.pyi`` in the current package

    Args:
        name: Module name (e.g. ``le_imgproc``). Output will be ``<name>.pyi``.
        extension: Label of the ``.so`` target (e.g. ``":le_imgproc.so"``).
        stub_deps: Labels of other ``.so`` targets this module imports at
            load time (needed so pybind11-stubgen can resolve cross-module
            references, e.g. ``le_lsd`` needs ``le_edge.so``).
        visibility: Bazel visibility for the generated stub.
    """

    # genrule srcs: the .so file + any dependency .so files
    srcs = [extension] + stub_deps

    # Build the command.  We need to pass the directory of each .so to the
    # runner script so it can set sys.path before importing the module.
    #
    # $(location X) gives us the path to the .so file; we use dirname in
    # the shell to extract the directory.
    so_dir_cmd = "$$(dirname $(location {ext}))".format(ext = extension)
    dep_dir_cmds = " ".join([
        "$$(dirname $(location {dep}))".format(dep = dep)
        for dep in stub_deps
    ])

    cmd = "$(location //tools/bazel:run_stubgen) {module} $(@D) {so_dir} {dep_dirs}".format(
        module = name,
        so_dir = so_dir_cmd,
        dep_dirs = dep_dir_cmds,
    )

    native.genrule(
        name = name + "_pyi",
        srcs = srcs,
        outs = [name + ".pyi"],
        cmd = cmd,
        tools = ["//tools/bazel:run_stubgen"],
        visibility = visibility if visibility else ["//visibility:public"],
    )
