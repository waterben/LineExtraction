"""Custom C++ toolchain configuration."""

load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")
load("@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl", "tool_path")
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")

all_compile_actions = [
    ACTION_NAMES.c_compile,
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.linkstamp_compile,
    ACTION_NAMES.assemble,
    ACTION_NAMES.preprocess_assemble,
    ACTION_NAMES.cpp_header_parsing,
    ACTION_NAMES.cpp_module_compile,
    ACTION_NAMES.cpp_module_codegen,
    ACTION_NAMES.clif_match,
    ACTION_NAMES.lto_backend,
]

all_link_actions = [
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

def _impl(ctx):
    if ctx.attr.compiler == "gcc":
        tool_paths = [
            tool_path(name = "gcc", path = "/usr/bin/gcc"),
            tool_path(name = "g++", path = "/usr/bin/g++"),
            tool_path(name = "cpp", path = "/usr/bin/cpp"),
            tool_path(name = "ar", path = "/usr/bin/ar"),
            tool_path(name = "nm", path = "/usr/bin/nm"),
            tool_path(name = "ld", path = "/usr/bin/ld"),
            tool_path(name = "as", path = "/usr/bin/as"),
            tool_path(name = "objcopy", path = "/usr/bin/objcopy"),
            tool_path(name = "objdump", path = "/usr/bin/objdump"),
            tool_path(name = "gcov", path = "/usr/bin/gcov"),
            tool_path(name = "strip", path = "/usr/bin/strip"),
            tool_path(name = "llvm-cov", path = "/bin/false"),
        ]
    elif ctx.attr.compiler == "clang":
        tool_paths = [
            tool_path(name = "gcc", path = "/usr/bin/clang"),
            tool_path(name = "g++", path = "/usr/bin/clang++"),
            tool_path(name = "cpp", path = "/usr/bin/clang-cpp"),
            tool_path(name = "ar", path = "/usr/bin/ar"),
            tool_path(name = "nm", path = "/usr/bin/nm"),
            tool_path(name = "ld", path = "/usr/bin/ld"),
            tool_path(name = "as", path = "/usr/bin/as"),
            tool_path(name = "objcopy", path = "/usr/bin/objcopy"),
            tool_path(name = "objdump", path = "/usr/bin/objdump"),
            tool_path(name = "gcov", path = "/usr/bin/llvm-cov"),
            tool_path(name = "strip", path = "/usr/bin/strip"),
            tool_path(name = "llvm-cov", path = "/usr/bin/llvm-cov"),
        ]
    else:
        fail("Unsupported compiler: " + ctx.attr.compiler)

    return cc_common.create_cc_toolchain_config_info(
        ctx = ctx,
        toolchain_identifier = ctx.attr.toolchain_identifier,
        host_system_name = ctx.attr.host_system_name,
        target_system_name = ctx.attr.target_system_name,
        target_cpu = ctx.attr.cpu,
        target_libc = ctx.attr.target_libc,
        compiler = ctx.attr.compiler,
        abi_version = ctx.attr.abi_version,
        abi_libc_version = ctx.attr.abi_libc_version,
        tool_paths = tool_paths,
    )

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {
        "cpu": attr.string(mandatory = True),
        "compiler": attr.string(mandatory = True),
        "toolchain_identifier": attr.string(mandatory = True),
        "host_system_name": attr.string(mandatory = True),
        "target_system_name": attr.string(mandatory = True),
        "target_libc": attr.string(mandatory = True),
        "abi_version": attr.string(mandatory = True),
        "abi_libc_version": attr.string(mandatory = True),
    },
    provides = [CcToolchainConfigInfo],
)
