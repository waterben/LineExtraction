#!/usr/bin/env python3
"""Interactive line segment detection demo for the le_lsd Python bindings.

Loads an image, lets the user choose a line segment detector (LsdCC, LsdBurns,
LsdFGioi, LsdEDLZ, etc.), run the pipeline, and display detected line segments
overlaid on the original image.

Usage::

    # Via Bazel (uses bundled windmill.jpg by default)
    bazel run //examples/lsd/python:lsd_demo

    # With a custom image
    bazel run //examples/lsd/python:lsd_demo -- /path/to/image.jpg

    # Directly (if le_lsd is importable)
    python lsd_demo.py [image_path]
"""

from __future__ import annotations

import argparse
import colorsys
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import le_lsd
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
# Available detectors
# ============================================================================

#: Detector classes keyed by display name.
DETECTORS: dict[str, dict[str, Any]] = {
    "CC (Connected Components)": {
        "class": "LsdCC",
        "desc": "Edge linking into connected components + split/fit",
    },
    "CP (CC with Patterns)": {
        "class": "LsdCP",
        "desc": "CC detection with pattern-based linking",
    },
    "Burns": {
        "class": "LsdBurns",
        "desc": "Classic Burns 'Extracting Straight Lines' algorithm",
    },
    "FBW (Fast Region Growing)": {
        "class": "LsdFBW",
        "desc": "Fast region growing from gradient-aligned seeds",
    },
    "FGioi (PLSD)": {
        "class": "LsdFGioi",
        "desc": "Probabilistic LSD using NFA validation",
    },
    "EDLines": {
        "class": "LsdEDLZ",
        "desc": "Edge Drawing Lines algorithm",
    },
    "EL (Edge Linking)": {
        "class": "LsdEL",
        "desc": "Sophisticated edge linking + NFA + sub-pixel",
    },
    "EP (Edge Pattern)": {
        "class": "LsdEP",
        "desc": "Edge pattern detection + split/fit",
    },
    "HoughP (Probabilistic Hough)": {
        "class": "LsdHoughP",
        "desc": "OpenCV Probabilistic Hough Transform",
    },
}

#: Type presets.
PRESETS: dict[str, dict[str, Any]] = {
    "float (default)": {"suffix": "", "label": "float"},
    "double (_f64)": {"suffix": "_f64", "label": "double"},
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
        return np.array(pil_img, dtype=np.uint8)
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

    out_dir = Path(tempfile.mkdtemp(prefix="lsd_demo_"))
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
        print("Usage: lsd_demo.py [image_path]")
        sys.exit(1)


def _generate_colors(n: int) -> list[tuple[float, float, float]]:
    """Generate n visually distinct colors using HSV spacing.

    :param n: Number of colors to generate.
    :type n: int
    :return: List of RGB tuples.
    :rtype: list[tuple[float, float, float]]
    """
    colors = []
    for i in range(max(n, 1)):
        hue = i / max(n, 1)
        rgb = colorsys.hsv_to_rgb(hue, 0.9, 0.9)
        colors.append(rgb)
    return colors


def _draw_segments(
    ax: plt.Axes,
    img: np.ndarray,
    segments: list[Any],
    title: str,
) -> None:
    """Draw line segments overlaid on the image.

    Each segment is drawn as a colored line between its endpoints.

    :param ax: Matplotlib axes to draw on.
    :type ax: matplotlib.axes.Axes
    :param img: Original grayscale image.
    :type img: numpy.ndarray
    :param segments: List of detected line segments (LineSegment objects).
    :type segments: list
    :param title: Title for the subplot.
    :type title: str
    """
    ax.imshow(img, cmap="gray", alpha=0.5)

    if len(segments) == 0:
        ax.set_title(f"{title}\n(0 segments)")
        ax.axis("off")
        return

    colors = _generate_colors(len(segments))

    for i, seg in enumerate(segments):
        ep = seg.end_points()  # (x1, y1, x2, y2)
        ax.plot(
            [ep[0], ep[2]],
            [ep[1], ep[3]],
            linewidth=1.5,
            color=colors[i % len(colors)],
            alpha=0.8,
        )

    ax.set_title(f"{title}\n({len(segments)} segments)")
    ax.axis("off")


# ============================================================================
# Main workflow
# ============================================================================


def main() -> None:
    """Run the interactive line segment detection demo."""
    parser = argparse.ArgumentParser(
        description="Interactive line segment detection demo for le_lsd."
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
    # Step 2: Select detector and preset
    # ------------------------------------------------------------------
    detector_names = list(DETECTORS.keys())
    detector_idx = _numbered_menu(
        "Select line segment detector", detector_names, default=0
    )
    detector_info = DETECTORS[detector_names[detector_idx]]

    preset_names = list(PRESETS.keys())
    preset_idx = _numbered_menu("Select type preset", preset_names, default=0)
    preset = PRESETS[preset_names[preset_idx]]

    # ------------------------------------------------------------------
    # Step 3: Instantiate detector
    # ------------------------------------------------------------------
    suffix = preset["suffix"]
    cls_name = detector_info["class"] + suffix

    cls = getattr(le_lsd, cls_name)
    det = cls()

    print(f"\n{'─' * 50}")
    print(f"  Detector:     {cls_name}")
    print(f"  Description:  {detector_info['desc']}")
    print(f"  Precision:    {preset['label']}")
    print(f"  Image:        {img_u8.shape[1]}x{img_u8.shape[0]} uint8")
    print(f"{'─' * 50}")

    # ------------------------------------------------------------------
    # Step 4: Detect
    # ------------------------------------------------------------------
    print("\nDetecting line segments... ", end="", flush=True)
    det.detect(img_u8)
    print("done.")

    segments = det.line_segments()
    num_segments = len(segments)
    print(f"\n  Found {num_segments} line segments.")

    if num_segments > 0:
        lengths = [seg.length for seg in segments]
        print(f"  Length range: [{min(lengths):.1f}, {max(lengths):.1f}]")
        print(f"  Mean length:  {sum(lengths) / len(lengths):.1f}")

    # ------------------------------------------------------------------
    # Step 5: Visualize results
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    fig.suptitle(
        f"{cls_name} — Line Segment Detection",
        fontsize=14,
        fontweight="bold",
    )

    # Left: Original image
    axes[0].imshow(img_u8, cmap="gray")
    axes[0].set_title("Original Image")
    axes[0].axis("off")

    # Center: Detected segments overlaid on image
    _draw_segments(axes[1], img_u8, segments, "Detected Segments")

    # Right: Segments on black background
    seg_img = np.zeros_like(img_u8)
    for seg in segments:
        ep = seg.end_points()  # (x1, y1, x2, y2)
        x1, y1, x2, y2 = int(ep[0]), int(ep[1]), int(ep[2]), int(ep[3])
        # Draw line on the segment image using Bresenham-like approach
        npts = max(abs(x2 - x1), abs(y2 - y1), 1)
        for t in range(npts + 1):
            frac = t / npts
            x = int(x1 + frac * (x2 - x1))
            y = int(y1 + frac * (y2 - y1))
            if 0 <= y < seg_img.shape[0] and 0 <= x < seg_img.shape[1]:
                seg_img[y, x] = 255

    axes[2].imshow(seg_img, cmap="gray")
    axes[2].set_title(f"Segment Map ({num_segments})")
    axes[2].axis("off")

    fig.tight_layout()

    p = _show_or_save(fig, "lsd_results.png")
    if p:
        saved_files.append(p)

    # ------------------------------------------------------------------
    # Step 6: Additional view — image data layers (if available)
    # ------------------------------------------------------------------
    try:
        desc = det.image_data_descriptor()
        data = det.image_data()
    except Exception:
        desc = []
        data = []

    if desc and data:
        num_layers = min(len(desc), 4)  # Show up to 4 layers
        fig2, axes2 = plt.subplots(1, num_layers, figsize=(4 * num_layers, 4))
        fig2.suptitle(
            f"{cls_name} — Auxiliary Image Data", fontsize=14, fontweight="bold"
        )

        if num_layers == 1:
            axes2 = [axes2]

        for i in range(num_layers):
            layer = data[i]
            layer_name = desc[i].name
            axes2[i].imshow(layer.astype(np.float64), cmap="hot")
            axes2[i].set_title(layer_name)
            axes2[i].axis("off")

        fig2.tight_layout()

        p = _show_or_save(fig2, "lsd_image_data.png")
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
