"""Stub generation runner for pybind11 extension modules.

Invoked as a Bazel tool (py_binary) during the build to generate .pyi
type stubs from compiled .so extensions.  Not intended for direct use;
called automatically by the ``le_pybind_stubgen`` genrule in pybind.bzl.

Usage (by Bazel genrule only)::

    run_stubgen <module_name> <output_dir> <so_dir> [<dep_so_dir> ...]
"""

from __future__ import annotations

import os
import sys


def main() -> None:
    """Generate a .pyi stub for a single pybind11 extension module."""
    if len(sys.argv) < 4:
        print(
            f"Usage: {sys.argv[0]} <module_name> <output_dir> <so_dir>"
            " [<dep_so_dir> ...]",
            file=sys.stderr,
        )
        sys.exit(1)

    module_name = sys.argv[1]
    output_dir = sys.argv[2]
    so_dirs = sys.argv[3:]

    # Make the .so importable by prepending its directories to sys.path
    for d in reversed(so_dirs):
        abs_d = os.path.abspath(d)
        if abs_d not in sys.path:
            sys.path.insert(0, abs_d)

    # Invoke pybind11-stubgen via its main() entry point (parses argv)
    from pybind11_stubgen import main as stubgen_main  # type: ignore[import-untyped]

    stubgen_argv = [
        module_name,
        "--output-dir",
        output_dir,
        "--numpy-array-remove-parameters",
        "--ignore-invalid-expressions",
        ".*",
    ]
    stubgen_main(stubgen_argv)

    expected = os.path.join(output_dir, f"{module_name}.pyi")
    if not os.path.isfile(expected):
        print(
            f"ERROR: pybind11-stubgen did not produce {expected}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
