"""Qt utilities for Bazel build"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def qt_cc_library(name, srcs = [], hdrs = [], ui_files = [], deps = [], **kwargs):
    """Build a Qt C++ library with MOC and UIC support.

    Args:
        name: Name of the target
        srcs: C++ source files
        hdrs: Header files (will be processed by MOC if they contain Q_OBJECT)
        ui_files: Qt .ui files (will be processed by UIC)
        deps: Dependencies
        **kwargs: Additional arguments passed to cc_library
    """

    # Generate MOC sources for headers
    moc_srcs = []
    for hdr in hdrs:
        if hdr.endswith(".h"):
            moc_name = "moc_" + hdr[:-2].replace("/", "_")
            native.genrule(
                name = moc_name,
                srcs = [hdr],
                outs = [moc_name + ".cpp"],
                cmd = "moc $(location " + hdr + ") -o $@",
                tools = [],
            )
            moc_srcs.append(moc_name + ".cpp")

    # Generate UI headers
    ui_hdrs = []
    for ui in ui_files:
        if ui.endswith(".ui"):
            ui_name = "ui_" + ui[:-3].replace("/", "_")
            native.genrule(
                name = ui_name,
                srcs = [ui],
                outs = [ui_name + ".h"],
                cmd = "uic $(location " + ui + ") -o $@",
                tools = [],
            )
            ui_hdrs.append(ui_name + ".h")

    # Create the library
    cc_library(
        name = name,
        srcs = srcs + moc_srcs,
        hdrs = hdrs + ui_hdrs,
        deps = deps,
        **kwargs
    )
