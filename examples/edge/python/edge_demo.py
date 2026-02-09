#!/usr/bin/env python3
"""Interactive edge detection demo for the le_edge Python bindings.

Loads an image, lets the user choose an edge source (Sobel/Scharr/Prewitt),
an edge segment detector (Drawing/Simple/Linking/Pattern), run the pipeline,
and display edge segments overlaid on the original image.

Usage::

    # Via Bazel (uses bundled windmill.jpg by default)
    bazel run //examples/edge/python:edge_demo

    # With a custom image
    bazel run //examples/edge/python:edge_demo -- /path/to/image.jpg

    # Directly (if le_edge is importable)
    python edge_demo.py [image_path]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import le_edge
import matplotlib
import numpy as np
from PIL import Image

# ---------------------------------------------------------------------------
# Matplotlib backend detection (Bazel hermetic Python may lack tkinter)
# ---------------------------------------------------------------------------
_INTERACTIVE = False
for _backend in ("TkAgg", "Qt5Agg", "GTK3Agg"):
    try:
        matplotlib.use(_backend, force=True)
        import matplotlib.pyplot as _plt_probe  # noqa: F401

        _plt_probe.switch_backend(_backend)
        _INTERACTIVE = True
        break
    except (ImportError, ModuleNotFoundError):
        continue

if not _INTERACTIVE:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

# ============================================================================
# Available edge sources and detectors
# ============================================================================

#: Edge source classes keyed by display name.
EDGE_SOURCES: dict[str, str] = {
    "Sobel": "EdgeSourceSobel",
    "Scharr": "EdgeSourceScharr",
    "Prewitt": "EdgeSourcePrewitt",
}

#: Edge segment detector classes keyed by display name.
DETECTORS: dict[str, str] = {
    "Drawing": "EsdDrawing",
    "Simple": "EsdSimple",
    "Linking": "EsdLinking",
    "Pattern": "EsdPattern",
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

    :param fig: The matplotlib figure to display.
    :type fig: matplotlib.figure.Figure
    :param filename: Basename for the saved PNG.
    :type filename: str
    :return: Path to the saved PNG, or ``None`` if shown interactively.
    :rtype: Path or None
    """
    if _INTERACTIVE:
        plt.show(block=False)
        plt.pause(0.1)
        return None

    out_dir = Path(tempfile.mkdtemp(prefix="edge_demo_"))
    out_path = out_dir / filename
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {out_path}")
    return out_path


def _open_images(paths: list[Path]) -> None:
    """Open saved image files with the default system viewer.

    :param paths: List of PNG file paths to open.
    :type paths: list[Path]
    """
    opener = shutil.which("xdg-open") or shutil.which("open")
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
        print("Usage: edge_demo.py [image_path]")
        sys.exit(1)


def _draw_segments(
    ax: plt.Axes,
    img: np.ndarray,
    segments: list[le_edge.EdgeSegment],
    points: list[int],
    title: str,
) -> None:
    """Draw edge segments overlaid on the image.

    Each segment is drawn in a different color.  Points are converted from
    linear indices to (x, y) coordinates.

    :param ax: Matplotlib axes to draw on.
    :type ax: matplotlib.axes.Axes
    :param img: Original grayscale image.
    :type img: numpy.ndarray
    :param segments: List of detected edge segments.
    :type segments: list[le_edge.EdgeSegment]
    :param points: List of linear pixel indices (shared by all segments).
    :type points: list[int]
    :param title: Title for the subplot.
    :type title: str
    """
    ax.imshow(img, cmap="gray", alpha=0.5)
    cols = img.shape[1]

    # Use a colormap for distinct segment colors
    cmap = plt.cm.get_cmap("tab20", max(len(segments), 1))

    for i, seg in enumerate(segments):
        seg_indices = points[seg.begin : seg.end]
        if not seg_indices:
            continue
        xs = [idx % cols for idx in seg_indices]
        ys = [idx // cols for idx in seg_indices]
        color = cmap(i % cmap.N)
        ax.plot(xs, ys, ".", markersize=0.8, color=color, alpha=0.9)

    ax.set_title(f"{title}\n({len(segments)} segments)")
    ax.axis("off")


# ============================================================================
# Main workflow
# ============================================================================


def main() -> None:
    """Run the interactive edge detection demo."""
    parser = argparse.ArgumentParser(
        description="Interactive edge detection demo for le_edge."
    )
    parser.add_argument(
        "image",
        nargs="?",
        default=None,
        help="Path to input image (default: windmill.jpg).",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Step 1: Load image
    # ------------------------------------------------------------------
    image_path: str = args.image if args.image else _resolve_default_image()
    img_u8 = _load_image(image_path)
    print(f"\nLoaded image: {image_path}  ({img_u8.shape[1]}x{img_u8.shape[0]})")

    saved_files: list[Path] = []

    # ------------------------------------------------------------------
    # Step 2: Select edge source, detector, and preset
    # ------------------------------------------------------------------
    source_names = list(EDGE_SOURCES.keys())
    source_idx = _numbered_menu("Select edge source", source_names, default=0)
    source_base = EDGE_SOURCES[source_names[source_idx]]

    detector_names = list(DETECTORS.keys())
    detector_idx = _numbered_menu(
        "Select edge segment detector", detector_names, default=0
    )
    detector_base = DETECTORS[detector_names[detector_idx]]

    preset_names = list(PRESETS.keys())
    preset_idx = _numbered_menu("Select type preset", preset_names, default=0)
    preset = PRESETS[preset_names[preset_idx]]

    min_pixels = _ask_int("Minimum segment length", default=10)

    # ------------------------------------------------------------------
    # Step 3: Instantiate and configure
    # ------------------------------------------------------------------
    suffix = preset["suffix"]
    es_cls_name = source_base + suffix
    esd_cls_name = detector_base + suffix

    # For ESD classes with MT=float (default, _16u, _f32) vs MT=double (_f64),
    # the ESD suffix follows the MT type, not the IT type.
    # Default/16u/f32 all use MT=float -> no suffix on ESD
    # f64 uses MT=double -> _f64 suffix on ESD
    esd_suffix = "_f64" if suffix == "_f64" else ""
    esd_cls_name = detector_base + esd_suffix

    es_cls = getattr(le_edge, es_cls_name)
    esd_cls = getattr(le_edge, esd_cls_name)

    es = es_cls()
    esd = esd_cls()
    esd.set_int("edge_min_pixels", min_pixels)

    # Convert image to the chosen dtype
    dtype = preset["dtype"]
    scale = preset["scale"]
    if dtype == np.uint8:
        img = img_u8.copy()
    elif dtype == np.uint16:
        img = (img_u8.astype(np.uint16) * (scale // 255)).astype(np.uint16)
    else:
        img = (img_u8.astype(dtype) / 255.0 * scale).astype(dtype)

    print(f"\n{'─' * 50}")
    print(f"  Edge Source:  {es_cls_name}")
    print(f"  Detector:     {esd_cls_name}")
    print(f"  Min Pixels:   {min_pixels}")
    print(f"  Image dtype:  {img.dtype}")
    print(f"  Image range:  [{img.min()}, {img.max()}]")
    print(f"{'─' * 50}")

    # ------------------------------------------------------------------
    # Step 4: Process
    # ------------------------------------------------------------------
    print("\nComputing edge source... ", end="", flush=True)
    es.process(img)
    print("done.")

    print("Detecting edge segments... ", end="", flush=True)
    esd.detect(es)
    print("done.")

    segments = esd.segments()
    points = esd.points()
    num_segments = len(segments)
    num_points = len(points)
    print(f"\n  Found {num_segments} segments with {num_points} total points.")

    # ------------------------------------------------------------------
    # Step 5: Visualize results
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(
        f"{es_cls_name} + {esd_cls_name}  (min_pixels={min_pixels})",
        fontsize=14,
        fontweight="bold",
    )

    # Top-left: Original image
    axes[0, 0].imshow(img_u8, cmap="gray")
    axes[0, 0].set_title("Original Image")
    axes[0, 0].axis("off")

    # Top-right: Magnitude map
    mag = es.magnitude()
    im_mag = axes[0, 1].imshow(mag.astype(np.float64), cmap="hot")
    axes[0, 1].set_title("Edge Magnitude")
    axes[0, 1].axis("off")
    fig.colorbar(im_mag, ax=axes[0, 1], fraction=0.046, pad=0.04)

    # Bottom-left: Direction map (only where edges are detected)
    direction = es.direction()
    im_dir = axes[1, 0].imshow(direction.astype(np.float64), cmap="hsv")
    axes[1, 0].set_title("Edge Direction")
    axes[1, 0].axis("off")
    fig.colorbar(im_dir, ax=axes[1, 0], fraction=0.046, pad=0.04)

    # Bottom-right: Detected segments overlaid on image
    _draw_segments(axes[1, 1], img_u8, segments, points, "Detected Segments")

    fig.tight_layout()

    p = _show_or_save(fig, "edge_results.png")
    if p:
        saved_files.append(p)

    # ------------------------------------------------------------------
    # Step 6: Additional view — hysteresis binary edge map
    # ------------------------------------------------------------------
    fig2, axes2 = plt.subplots(1, 2, figsize=(12, 5))
    fig2.suptitle("Edge Maps", fontsize=14, fontweight="bold")

    # Hysteresis binary
    hyst_bin = es.hysteresis_binary()
    axes2[0].imshow(hyst_bin, cmap="gray")
    axes2[0].set_title("Hysteresis Binary")
    axes2[0].axis("off")

    # Segment overlay on black background
    seg_img = np.zeros_like(img_u8)
    cols = img_u8.shape[1]
    for seg in segments:
        seg_indices = points[seg.begin : seg.end]
        for idx in seg_indices:
            row = idx // cols
            col = idx % cols
            if 0 <= row < seg_img.shape[0] and 0 <= col < seg_img.shape[1]:
                seg_img[row, col] = 255
    axes2[1].imshow(seg_img, cmap="gray")
    axes2[1].set_title(f"Edge Segments ({num_segments})")
    axes2[1].axis("off")

    fig2.tight_layout()

    p = _show_or_save(fig2, "edge_maps.png")
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
