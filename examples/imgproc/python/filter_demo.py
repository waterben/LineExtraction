#!/usr/bin/env python3
"""Interactive filter demo for the le_imgproc Python bindings.

Loads an image, lets the user choose a gradient filter and type preset,
configure kernel size, run the filter, and display the results side by side
using matplotlib.

Usage::

    # Via Bazel (uses bundled windmill.jpg by default)
    bazel run //examples/imgproc/python:filter_demo

    # With a custom image
    bazel run //examples/imgproc/python:filter_demo -- /path/to/image.jpg

    # Directly (if le_imgproc is importable)
    python filter_demo.py [image_path]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import le_imgproc
import matplotlib
import numpy as np
from PIL import Image

# Try to use an interactive matplotlib backend.  Bazel's hermetic Python
# typically lacks tkinter, so the default "Agg" is non-interactive.
# matplotlib.use() can succeed but the backend may still fail at import
# time, so we force an actual import test via matplotlib.pyplot.switch_backend().
_INTERACTIVE = False
for _backend in ("TkAgg", "Qt5Agg", "GTK3Agg"):
    try:
        matplotlib.use(_backend, force=True)
        # Force the backend to actually load — this triggers the real ImportError
        # if the underlying toolkit (e.g. _tkinter) is missing.
        import matplotlib.pyplot as _plt_probe  # noqa: F401

        _plt_probe.switch_backend(_backend)
        _INTERACTIVE = True
        break
    except (ImportError, ModuleNotFoundError):
        continue

if not _INTERACTIVE:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  — must import after backend selection


# ============================================================================
# Available filters and type presets
# ============================================================================

#: Gradient filter classes available in ``le_imgproc``, keyed by display name.
FILTERS: dict[str, str] = {
    "Sobel": "SobelGradient",
    "Scharr": "ScharrGradient",
    "Prewitt": "PrewittGradient",
}

#: Type presets with their suffix, numpy dtype, and description.
PRESETS: dict[str, dict[str, Any]] = {
    "uint8  (default)": {"suffix": "", "dtype": np.uint8, "scale": 255},
    "uint16 (_16u)": {"suffix": "_16u", "dtype": np.uint16, "scale": 65535},
    "float32 (_f32)": {"suffix": "_f32", "dtype": np.float32, "scale": 1.0},
    "float64 (_f64)": {"suffix": "_f64", "dtype": np.float64, "scale": 1.0},
}


# ============================================================================
# Helpers
# ============================================================================


def _numbered_menu(title: str, items: list[str], default: int = 0) -> int:
    """Print a numbered menu and return the user's selection index.

    :param title: Header text for the menu.
    :type title: str
    :param items: List of option labels.
    :type items: list[str]
    :param default: Default selection index (0-based).
    :type default: int
    :return: Selected index (0-based).
    :rtype: int
    """
    print(f"\n{'─' * 50}")
    print(f"  {title}")
    print(f"{'─' * 50}")
    for i, item in enumerate(items):
        marker = " *" if i == default else ""
        print(f"  [{i + 1}] {item}{marker}")
    print()

    while True:
        raw = input(f"  Choice [default={default + 1}]: ").strip()
        if not raw:
            return default
        try:
            choice = int(raw)
            if 1 <= choice <= len(items):
                return choice - 1
        except ValueError:
            pass
        print(f"  Please enter a number between 1 and {len(items)}.")


def _ask_int(prompt: str, default: int, valid: list[int] | None = None) -> int:
    """Ask the user for an integer value.

    :param prompt: Prompt text.
    :type prompt: str
    :param default: Default value if the user presses Enter.
    :type default: int
    :param valid: Optional list of allowed values.
    :type valid: list[int] or None
    :return: The chosen integer.
    :rtype: int
    """
    while True:
        raw = input(f"  {prompt} [default={default}]: ").strip()
        if not raw:
            return default
        try:
            val = int(raw)
            if valid is None or val in valid:
                return val
            print(f"  Allowed values: {valid}")
        except ValueError:
            print("  Please enter an integer.")


def _load_image(path: str) -> np.ndarray:
    """Load an image as grayscale uint8 via Pillow.

    :param path: Path to the image file.
    :type path: str
    :return: Grayscale image as 2-D uint8 array.
    :rtype: numpy.ndarray
    :raises SystemExit: If the image cannot be loaded.
    """
    try:
        pil_img = Image.open(path).convert("L")
        return np.asarray(pil_img, dtype=np.uint8)
    except Exception as exc:
        print(f"Error: could not load image '{path}': {exc}")
        sys.exit(1)


def _show_or_save(fig: matplotlib.figure.Figure, filename: str) -> Path | None:
    """Show a figure interactively or save it to a temp PNG.

    When an interactive backend is available the figure is shown with
    ``plt.show(block=False)``.  Otherwise the figure is saved to a
    temporary directory and later opened via ``xdg-open``.

    :param fig: The matplotlib figure to display.
    :type fig: matplotlib.figure.Figure
    :param filename: Basename for the saved PNG (e.g. ``"input.png"``).
    :type filename: str
    :return: Path to the saved PNG, or ``None`` if shown interactively.
    :rtype: Path or None
    """
    if _INTERACTIVE:
        plt.show(block=False)
        plt.pause(0.1)
        return None

    out_dir = Path(tempfile.mkdtemp(prefix="filter_demo_"))
    out_path = out_dir / filename
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {out_path}")
    return out_path


def _open_images(paths: list[Path]) -> None:
    """Open saved image files with the default system viewer.

    :param paths: List of PNG file paths to open.
    :type paths: list[Path]
    """
    opener = shutil.which("xdg-open") or shutil.which("open")  # Linux / macOS
    if not opener:
        print("\n  No image viewer found. Open the files manually:")
        for p in paths:
            print(f"    {p}")
        return

    for p in paths:
        subprocess.Popen(  # noqa: S603
            [opener, str(p)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def _resolve_default_image() -> str:
    """Try to find windmill.jpg via lsfm.data.TestImages.

    :return: Path string to windmill.jpg.
    :rtype: str
    :raises SystemExit: If the default test image is not found.
    """
    try:
        from lsfm.data import TestImages

        return str(TestImages().windmill())
    except (ImportError, FileNotFoundError):
        print("Error: no image specified and default windmill.jpg not found.")
        print("Usage: filter_demo.py [image_path]")
        sys.exit(1)


# ============================================================================
# Main workflow
# ============================================================================


def main() -> None:
    """Run the interactive filter demo."""
    parser = argparse.ArgumentParser(
        description="Interactive gradient filter demo for le_imgproc."
    )
    parser.add_argument(
        "image",
        nargs="?",
        default=None,
        help="Path to input image (default: windmill.jpg).",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Step 1: Load and display the input image
    # ------------------------------------------------------------------
    image_path: str = args.image if args.image else _resolve_default_image()
    img_u8 = _load_image(image_path)

    print(f"\nLoaded image: {image_path}  ({img_u8.shape[1]}x{img_u8.shape[0]})")

    saved_files: list[Path] = []

    fig_input, ax_input = plt.subplots(1, 1, figsize=(6, 5))
    ax_input.imshow(img_u8, cmap="gray")
    ax_input.set_title(f"Input Image ({img_u8.shape[1]}x{img_u8.shape[0]})")
    ax_input.axis("off")
    fig_input.tight_layout()
    p = _show_or_save(fig_input, "input.png")
    if p:
        saved_files.append(p)

    # ------------------------------------------------------------------
    # Step 2: Select filter, preset, and kernel size
    # ------------------------------------------------------------------
    filter_names = list(FILTERS.keys())
    filter_idx = _numbered_menu("Select gradient filter", filter_names, default=0)
    filter_base_name = FILTERS[filter_names[filter_idx]]

    preset_names = list(PRESETS.keys())
    preset_idx = _numbered_menu("Select type preset", preset_names, default=0)
    preset = PRESETS[preset_names[preset_idx]]

    kernel_size = _ask_int("Kernel size (1, 3, 5, 7)", default=3, valid=[1, 3, 5, 7])

    # ------------------------------------------------------------------
    # Step 3: Instantiate filter and convert image
    # ------------------------------------------------------------------
    class_name = filter_base_name + preset["suffix"]
    filter_cls = getattr(le_imgproc, class_name)
    grad = filter_cls()
    grad.set_int("grad_kernel_size", kernel_size)

    # Convert the uint8 image to the chosen dtype.
    dtype = preset["dtype"]
    scale = preset["scale"]
    if dtype == np.uint8:
        img = img_u8.copy()
    elif dtype == np.uint16:
        img = (img_u8.astype(np.uint16) * (scale // 255)).astype(np.uint16)
    else:
        img = (img_u8.astype(dtype) / 255.0 * scale).astype(dtype)

    print(f"\n{'─' * 50}")
    print(f"  Filter:      {class_name}")
    print(f"  Kernel size: {kernel_size}")
    print(f"  Image dtype: {img.dtype}")
    print(f"  Image range: [{img.min()}, {img.max()}]")
    print(f"{'─' * 50}")

    # ------------------------------------------------------------------
    # Step 4: Run filter and display results
    # ------------------------------------------------------------------
    print("\nProcessing... ", end="", flush=True)
    grad.process(img)
    print("done.")

    mag = grad.magnitude()
    direction = grad.direction()
    gx = grad.gx()
    gy = grad.gy()

    # Build a 2x2 result grid
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    fig.suptitle(
        f"{class_name}  (kernel={kernel_size})", fontsize=14, fontweight="bold"
    )

    titles = ["Magnitude", "Direction", "Gradient X", "Gradient Y"]
    images = [mag, direction, gx, gy]
    cmaps = ["hot", "hsv", "gray", "gray"]

    for ax, title, data, cmap in zip(axes.flat, titles, images, cmaps):
        im = ax.imshow(data.astype(np.float64), cmap=cmap)
        ax.set_title(title)
        ax.axis("off")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.tight_layout()

    p = _show_or_save(fig, "results.png")
    if p:
        saved_files.append(p)

    if saved_files:
        _open_images(saved_files)
        print("\nPress Enter to exit...")
        input()
    elif _INTERACTIVE:
        plt.ioff()
        print("\nClose the plot window(s) to exit.")
        plt.show()


if __name__ == "__main__":
    main()
