# Rerun Visualization Setup

[Rerun](https://rerun.io/) is used throughout this project to visualize images, line detections, and timelines.
This guide covers how to install the Rerun viewer and run the included demos on different platforms.

## Table of Contents

- [Python SDK vs C++ SDK](#python-sdk-vs-c-sdk)
- [Installing the Rerun Viewer](#installing-the-rerun-viewer)
- [Platform Setup](#platform-setup)
  - [Native Linux](#native-linux)
  - [WSL (Windows Subsystem for Linux)](#wsl-windows-subsystem-for-linux)
  - [Docker / Headless](#docker--headless)
- [Using Rerun in Jupyter Notebooks](#using-rerun-in-jupyter-notebooks)
- [Running the C++ Demo](#running-the-c-demo)
- [Running the Python Demos](#running-the-python-demos)
- [Bazel Targets Reference](#bazel-targets-reference)
- [Viewer Modes](#viewer-modes)

---

## Python SDK vs C++ SDK

The project uses **two separate, independent Rerun SDKs** — one for Python and one for C++.
They share the same wire protocol and recording format (both are version 0.29.x), so data logged
from either can be viewed in the same Rerun viewer.

| | Python SDK | C++ SDK |
|---|---|---|
| **Package** | `rerun-sdk[notebook]>=0.29.0` (pip) | `rerun_cpp_sdk.zip` 0.29.2 (cmake) |
| **Where installed** | `.venv/` (always available) | compiled on demand by Bazel |
| **Bazel dep** | `//tools/bazel/third_party:rerun_sdk` | `@rerun_sdk//:rerun_sdk` |
| **First-build cost** | none (pip install) | ~15 min (compiles Apache Arrow 18) |
| **Jupyter notebooks** | ✅ works directly | ✗ not needed |
| **Python scripts / `py_binary`** | ✅ | ✗ not needed |
| **C++ binaries / `cc_binary`** | ✗ not needed | ✅ |

> **The two Bazel target names look similar but are unrelated:**
>
> - `//tools/bazel/third_party:rerun_sdk` → Python `py_library` wrapper around `@pip//rerun_sdk`
> - `@rerun_sdk//:rerun_sdk` → C++ `cmake()` target that compiles librerun_sdk.a + libArrow.a

### No opt-in switch needed

Bazel only builds what a target explicitly depends on.
Adding a dependency on `@rerun_sdk//:rerun_sdk` in a `cc_binary` is the only thing that triggers
the C++ Rerun + Arrow compile.  All Python targets and notebooks are completely unaffected — they
never see the C++ build.

If you want to test without C++ Rerun, simply do not build `//examples/other:rerun_cpp_demo`
(or any target you add that depends on `@rerun_sdk//:rerun_sdk`).

---

## Installing the Rerun Viewer

The Rerun viewer version must match the SDK version used in the project (`>= 0.29.0`).

**Recommended — install via pip (matches the Python SDK already in the project):**

```bash
# Activate the project venv first
source .venv/bin/activate

pip install rerun-sdk

# Verify version
rerun --version
```

**Alternative — via cargo (Rust toolchain required):**

```bash
cargo install rerun-cli --locked
```

**Alternative — download the pre-built binary:**

```bash
# Example for 0.29.2; adjust for the latest release
curl -L https://github.com/rerun-io/rerun/releases/download/0.29.2/rerun-cli--linux-x64.tar.gz \
  | tar xz -C ~/.local/bin
```

---

## Platform Setup

### Native Linux

No additional setup is required. The Rerun viewer uses the system's X11 or Wayland compositor directly.

```bash
# Start the viewer (blocks until closed; run in a separate terminal or background it)
rerun &

# Or let the SDK spawn it automatically (Python only)
```

If you want to force a specific display backend:

```bash
# Force X11
WAYLAND_DISPLAY= rerun &

# Force Wayland
DISPLAY= rerun &
```

---

### WSL (Windows Subsystem for Linux)

WSL has no compositor by default, so the Rerun viewer cannot open a native window unless a compatible X or Wayland bridge is configured.
The easiest option for most users is the **web viewer**.

#### Option A — Web Viewer (recommended for WSL, no extra setup needed)

Start Rerun in web-serve mode and open the URL in your Windows browser:

```bash
rerun --serve &
```

Rerun prints a URL like `http://localhost:9090`. Open it in any browser on the Windows host.

Then run the demo (it connects to the viewer over gRPC, independent of the display):

```bash
bazel run //examples/other:rerun_cpp_demo
```

#### Option B — WSLg (Windows 11 with WSL 2 ≥ 5.10.16)

WSLg provides a built-in Wayland compositor bridge. No configuration is needed — just run:

```bash
rerun &
```

Check that WSLg is active:

```bash
echo $WAYLAND_DISPLAY   # should print wayland-0 or similar
ls /mnt/wslg/           # directory should exist
```

If the variable is empty, WSLg is not available and you need Option A or C.

#### Option C — X Server (Windows 10 / WSL 1)

Install an X server on Windows (e.g., [VcXsrv](https://sourceforge.net/projects/vcxsrv/),
[X410](https://x410.app/), or [MobaXterm](https://mobaxterm.mobatek.net/)) and configure the `DISPLAY` variable in WSL:

```bash
# Typically set in .project_env automatically by the setup script.
# If not, set manually:
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0

# Force X11 (disable Wayland so the viewer does not try the missing compositor)
export WAYLAND_DISPLAY=

rerun &
```

Add these exports to `~/.bashrc` or `.project_env` to persist across sessions.

---

### Docker / Headless

Inside Docker containers (including the project DevContainer) there is no display. Use the web viewer:

```bash
rerun --serve --bind 0.0.0.0 &
```

Then open the printed URL from your host browser.
Forward port 9090 if needed (VS Code Dev Containers forwards it automatically when you open the panel).

> **Note:** The C++ and Python SDK both connect over gRPC (port 9876 by default), which works identically
> in web-viewer mode. No code changes are needed.

---

## Using Rerun in Jupyter Notebooks

Jupyter notebooks use the **Python SDK only** — no C++ build is required.
The `rerun-sdk[notebook]` package is already installed in `.venv` via `pyproject.toml`.

### Quick start

```python
import rerun as rr
import rerun.notebook as rr_nb
import numpy as np

# Initialize a recording for the notebook (inline renderer)
rr.init("my_notebook_demo")
rr_nb.as_html()  # renders the viewer inline in the notebook output cell

# Log data
rr.set_time_sequence("frame", 0)
rr.log("image", rr.Image(np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)))
```

### Connecting to the standalone viewer (optional)

Instead of the inline renderer you can stream to a running viewer:

```python
import rerun as rr
rr.init("my_notebook_demo")
rr.connect_grpc()   # connects to rerun viewer at localhost:9876

# Then start the viewer (in a terminal or at the top of the notebook):
# rerun --serve &   (WSL / headless)
# rerun &           (native Linux with display)
```

### Available rerun notebook

A ready-made Rerun notebook for LSD visualization is included:

```bash
examples/notebooks/rerun_lsd_demo.ipynb
```

Open it in VS Code or JupyterLab — the kernel is the project `.venv`, same as all other notebooks.
See [docs/JUPYTER.md](JUPYTER.md) for general notebook setup instructions.

### Display in WSL / headless

The inline notebook renderer (`rr_nb.as_html()`) renders in the browser cell output — **no Rerun viewer
process needed at all**. This works in any environment (WSL, Docker, SSH) as long as a browser can
show the notebook.

---

## Running the C++ Demo

The demo logs a synthetic gradient image with a rotating star line pattern and grid keypoints.

### 1. Start the viewer

Choose the appropriate method for your platform (see above). For the web viewer:

```bash
rerun --serve &
# Open http://localhost:9090 in your browser
```

### 2. Build and run

```bash
bazel run //examples/other:rerun_cpp_demo
```

The demo streams 60 animated frames to `rerun://localhost:9876` (default gRPC endpoint)
and exits. The viewer retains the recording in memory — use the timeline slider to replay it.

### Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `Could not find wayland compositor` | No Wayland session | Use `WAYLAND_DISPLAY= rerun &` or the web viewer |
| `Failed to flush` | Viewer not running or wrong port | Start the viewer before running the demo |
| `Connection refused` / `exit_on_failure` | gRPC port 9876 not reachable | Ensure `rerun` or `rerun --serve` is running |
| Build takes ~15 min on first run | Arrow 18.0.0 is compiled from source | Expected; subsequent builds use the Bazel cache |

---

## Bazel Targets Reference

### Python targets

All Python Rerun targets depend on `//tools/bazel/third_party:rerun_sdk`
(the pip-based Python `py_library`). They are always buildable — no extra compile step.

```python
# In a BUILD.bazel py_binary:
deps = [
    "//tools/bazel/third_party:rerun_sdk",  # Python SDK (pip wheel wrapper)
    "@pip//numpy",
]
```

#### Why the wrapper target exists — `.pth` workaround

The `rerun-sdk` pip wheel installs `import rerun` via a `.pth` file that adds the package
subdirectory to `sys.path` at interpreter startup. Bazel's `rules_python` does **not** process
`.pth` files, so a bare `@pip//rerun_sdk` dep would result in `ModuleNotFoundError: No module named 'rerun'`.

The wrapper target `//tools/bazel/third_party:rerun_sdk` adds
[`rerun_pth_fix.py`](../tools/bazel/third_party/rerun_pth_fix.py) as a `py_library` dep, which
patches `sys.path` at import time — equivalent to what the `.pth` file does in a normal pip environment.

The Python demo scripts guard this import so they work both inside and outside Bazel:

```python
try:
    import rerun_pth_fix as _  # Bazel .pth workaround — not available outside Bazel
except ImportError:
    pass  # Running directly with pip-installed rerun-sdk: .pth is processed normally
import rerun as rr
```

| Target | Description |
|--------|---------|
| `//examples/other/python:rerun_general_demo` | Synthetic data demo |
| `//examples/lsd/python:rerun_lsd_demo` | LSD line detection visualization |

### C++ targets

C++ targets depend on `@rerun_sdk//:rerun_sdk` (the cmake-built library).
The first build compiles Apache Arrow from source (~15 min); subsequent builds use the Bazel cache.

```python
# In a BUILD.bazel cc_binary:
deps = [
    "@rerun_sdk//:rerun_sdk",  # C++ SDK (cmake-built)
    # ... other deps
]
```

| Target | Description |
|--------|-------------|
| `//examples/other:rerun_cpp_demo` | Gradient image + star line strips + keypoints |

---

## Running the Python Demos

Python demos can be run via Bazel or directly as scripts (if `rerun-sdk` and `le_lsd` are
installed in the active Python environment).

```bash
# --- Via Bazel ---

# Native viewer (requires a display)
bazel run //examples/other/python:rerun_general_demo

# Web viewer (WSL / headless)
bazel run //examples/other/python:rerun_general_demo -- --serve

# LSD line detector demo with web viewer
bazel run //examples/lsd/python:rerun_lsd_demo -- --serve

# Save recording to file (no viewer needed)
bazel run //examples/other/python:rerun_general_demo -- --save /tmp/output.rrd

# --- Direct execution (pip environment, .venv active) ---
python examples/other/python/rerun_general_demo.py --serve
python examples/lsd/python/rerun_lsd_demo.py --serve
```

> **Note:** Direct execution requires the Python bindings to be importable.
> Build them first (`bazel build //libs/...`) and set `PYTHONPATH` to include the Bazel output,
> or use the `.venv` with installed packages.

Open saved `.rrd` files later:

```bash
rerun /tmp/output.rrd
```

---

## Viewer Modes

| Mode | Command | Use case |
|------|---------|----------|
| Native window | `rerun` | Native Linux with display |
| Web viewer | `rerun --serve` | WSL, Docker, headless, remote |
| Save to file | _(SDK-side)_ `--save <path>.rrd` | Offline analysis, CI |
| Connect (gRPC) | _(default)_ `rec.connect_grpc()` | C++ and Python SDK, any platform |

The gRPC port (default **9876**) and web port (default **9090**) can be changed:

```bash
rerun --serve --port 9091 --grpc-port 9877
```

Update the SDK call accordingly:

```cpp
// C++
rec.connect_grpc("rerun+http://localhost:9877").exit_on_failure();
```

```python
# Python
rr.connect_grpc("rerun+http://localhost:9877")
```
