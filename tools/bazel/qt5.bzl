"""Qt5 build rules for Bazel.

This module provides rules for generating Qt MOC and UIC files.

Usage:
    load("//tools/bazel:qt5.bzl", "qt5_moc", "qt5_uic", "qt5_cc_library")

    # Generate moc files for headers with Q_OBJECT
    qt5_moc(
        name = "mylib_moc",
        hdrs = ["myclass.h"],
    )

    # Generate ui_*.h files from .ui files
    qt5_uic(
        name = "mylib_uic",
        srcs = ["mywidget.ui"],
    )
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

# Qt5 system paths (Ubuntu/Debian x86_64)
QT5_INCLUDE_PATH = "/usr/include/x86_64-linux-gnu/qt5"
QT5_MOC = "/usr/lib/qt5/bin/moc"
QT5_UIC = "/usr/lib/qt5/bin/uic"

# System include paths for Qt5
QT5_COPTS = [
    "-isystem/usr/include/x86_64-linux-gnu/qt5",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtCore",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtGui",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtWidgets",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtOpenGL",
    "-isystem/usr/include/x86_64-linux-gnu/qt5/QtPrintSupport",
    "-fPIC",
]

def _qt5_moc_impl(ctx):
    """Generate moc_*.cpp files from headers containing Q_OBJECT."""
    outputs = []
    for hdr in ctx.files.hdrs:
        # Output moc file name: foo.h -> moc_foo.cpp
        moc_name = "moc_" + hdr.basename.rsplit(".", 1)[0] + ".cpp"
        moc_out = ctx.actions.declare_file(moc_name)
        outputs.append(moc_out)

        # Build include paths
        include_args = []
        for inc in ctx.attr.includes:
            include_args.extend(["-I", inc])
        include_args.extend(["-I", ctx.label.package])
        include_args.extend(["-I", QT5_INCLUDE_PATH])
        include_args.extend(["-I", QT5_INCLUDE_PATH + "/QtCore"])
        include_args.extend(["-I", QT5_INCLUDE_PATH + "/QtGui"])
        include_args.extend(["-I", QT5_INCLUDE_PATH + "/QtWidgets"])

        ctx.actions.run(
            inputs = [hdr] + ctx.files.deps,
            outputs = [moc_out],
            executable = QT5_MOC,
            arguments = include_args + [
                "-o",
                moc_out.path,
                hdr.path,
            ],
            mnemonic = "QtMoc",
            progress_message = "Generating MOC for %s" % hdr.short_path,
        )

    return [
        DefaultInfo(files = depset(outputs)),
        OutputGroupInfo(moc_files = depset(outputs)),
    ]

qt5_moc = rule(
    implementation = _qt5_moc_impl,
    attrs = {
        "hdrs": attr.label_list(
            allow_files = [".h", ".hpp"],
            doc = "Header files containing Q_OBJECT that need moc processing",
        ),
        "deps": attr.label_list(
            allow_files = [".h", ".hpp"],
            doc = "Additional header dependencies",
        ),
        "includes": attr.string_list(
            doc = "Additional include directories",
        ),
    },
    doc = "Generate moc_*.cpp files from Qt headers with Q_OBJECT",
)

def _qt5_uic_impl(ctx):
    """Generate ui_*.h files from .ui XML files."""
    outputs = []
    for ui in ctx.files.srcs:
        # Output ui file name: foo.ui -> ui_foo.h
        ui_name = "ui_" + ui.basename.rsplit(".", 1)[0] + ".h"
        ui_out = ctx.actions.declare_file(ui_name)
        outputs.append(ui_out)

        ctx.actions.run(
            inputs = [ui],
            outputs = [ui_out],
            executable = QT5_UIC,
            arguments = [
                "-o",
                ui_out.path,
                ui.path,
            ],
            mnemonic = "QtUic",
            progress_message = "Generating UIC for %s" % ui.short_path,
        )

    return [
        DefaultInfo(files = depset(outputs)),
        OutputGroupInfo(uic_files = depset(outputs)),
        CcInfo(
            compilation_context = cc_common.create_compilation_context(
                headers = depset(outputs),
                includes = depset([outputs[0].dirname] if outputs else []),
            ),
        ),
    ]

qt5_uic = rule(
    implementation = _qt5_uic_impl,
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".ui"],
            mandatory = True,
            doc = "Qt .ui files to process",
        ),
    },
    doc = "Generate ui_*.h files from Qt Designer .ui files",
)

def qt5_cc_library(
        name,
        srcs = [],
        hdrs = [],
        ui_srcs = [],
        moc_hdrs = [],
        copts = [],
        deps = [],
        includes = [],
        **kwargs):
    """A cc_library with Qt5 MOC and UIC support.

    Args:
        name: Target name.
        srcs: Regular C++ source files.
        hdrs: Regular header files.
        ui_srcs: .ui files to process with UIC.
        moc_hdrs: Header files with Q_OBJECT to process with MOC.
        copts: Additional compiler options.
        deps: Library dependencies.
        includes: Additional include paths.
        **kwargs: Additional args passed to cc_library.
    """
    gen_srcs = []
    gen_deps = []

    # Generate MOC files
    if moc_hdrs:
        qt5_moc(
            name = name + "_moc",
            hdrs = moc_hdrs,
            includes = includes,
        )
        gen_srcs.append(":" + name + "_moc")

    # Generate UIC files
    if ui_srcs:
        qt5_uic(
            name = name + "_uic",
            srcs = ui_srcs,
        )
        gen_deps.append(":" + name + "_uic")

    # Combine all sources
    cc_library(
        name = name,
        srcs = srcs + gen_srcs,
        hdrs = hdrs,
        copts = QT5_COPTS + copts,
        deps = deps + gen_deps + [
            "@qt5//:qt5_core",
            "@qt5//:qt5_gui",
            "@qt5//:qt5_widgets",
        ],
        includes = includes,
        **kwargs
    )

def qt5_cc_binary(
        name,
        srcs = [],
        hdrs = [],
        ui_srcs = [],
        moc_hdrs = [],
        copts = [],
        deps = [],
        includes = [],
        **kwargs):
    """A cc_binary with Qt5 MOC and UIC support.

    Args:
        name: Target name.
        srcs: Regular C++ source files.
        hdrs: Header files (non-MOC).
        ui_srcs: .ui files to process with UIC.
        moc_hdrs: Header files with Q_OBJECT to process with MOC.
        copts: Additional compiler options.
        deps: Library dependencies.
        includes: Additional include paths.
        **kwargs: Additional args passed to cc_binary.
    """
    gen_srcs = []
    gen_deps = []

    # Generate MOC files
    if moc_hdrs:
        qt5_moc(
            name = name + "_moc",
            hdrs = moc_hdrs,
            includes = includes,
        )
        gen_srcs.append(":" + name + "_moc")

    # Generate UIC files
    if ui_srcs:
        qt5_uic(
            name = name + "_uic",
            srcs = ui_srcs,
        )
        gen_deps.append(":" + name + "_uic")

    native.cc_binary(
        name = name,
        srcs = srcs + gen_srcs,
        copts = QT5_COPTS + copts,
        deps = deps + gen_deps + [
            "@qt5//:qt5_core",
            "@qt5//:qt5_gui",
            "@qt5//:qt5_widgets",
        ],
        **kwargs
    )
