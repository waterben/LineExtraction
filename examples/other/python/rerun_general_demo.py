#!/usr/bin/env python3
"""General Rerun.io visualization demo with synthetic data.

Demonstrates core Rerun concepts — image logging, 2D line strips, 2D points,
text annotations, and timelines — without depending on any le_* bindings.

Usage::

    # Via Bazel
    bazel run //examples/other/python:rerun_general_demo

    # Directly (if rerun-sdk is installed in the active environment)
    python rerun_general_demo.py
"""

from __future__ import annotations

import argparse
import colorsys
import math

import numpy as np

try:
    import rerun_pth_fix as _  # noqa: F401  (Bazel .pth workaround — not available outside Bazel)
except ImportError:
    pass
import rerun as rr


# ============================================================================
# Synthetic data generators
# ============================================================================


def _make_gradient_image(width: int = 640, height: int = 480) -> np.ndarray:
    """Create a grayscale gradient image with a circular highlight.

    :param width: Image width in pixels.
    :type width: int
    :param height: Image height in pixels.
    :type height: int
    :return: Grayscale uint8 image of shape ``(height, width)``.
    :rtype: numpy.ndarray
    """
    y, x = np.mgrid[:height, :width]
    # Horizontal gradient as base
    base = (x / width * 200).astype(np.float64)
    # Add radial highlight in center
    cx, cy = width / 2, height / 2
    r = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    highlight = np.clip(120 - r * 0.5, 0, 120)
    return np.clip(base + highlight, 0, 255).astype(np.uint8)


def _make_star_lines(
    cx: float, cy: float, radius: float, n_rays: int = 12
) -> list[list[list[float]]]:
    """Generate line strips forming a star pattern.

    :param cx: Center x coordinate.
    :type cx: float
    :param cy: Center y coordinate.
    :type cy: float
    :param radius: Outer radius of the star.
    :type radius: float
    :param n_rays: Number of rays.
    :type n_rays: int
    :return: List of line strips, each strip is ``[[x1, y1], [x2, y2]]``.
    :rtype: list[list[list[float]]]
    """
    strips = []
    for i in range(n_rays):
        angle = 2 * math.pi * i / n_rays
        x_end = cx + radius * math.cos(angle)
        y_end = cy + radius * math.sin(angle)
        strips.append([[cx, cy], [x_end, y_end]])
    return strips


def _make_random_segments(
    width: int, height: int, n: int = 30, rng: np.random.Generator | None = None
) -> list[list[list[float]]]:
    """Generate random line segments within image bounds.

    :param width: Image width.
    :type width: int
    :param height: Image height.
    :type height: int
    :param n: Number of segments.
    :type n: int
    :param rng: Optional numpy random generator.
    :type rng: numpy.random.Generator or None
    :return: List of line strips, each strip is ``[[x1, y1], [x2, y2]]``.
    :rtype: list[list[list[float]]]
    """
    if rng is None:
        rng = np.random.default_rng(42)
    endpoints = rng.random((n, 2, 2))
    endpoints[:, :, 0] *= width
    endpoints[:, :, 1] *= height
    return [
        [[float(s[0, 0]), float(s[0, 1])], [float(s[1, 0]), float(s[1, 1])]]
        for s in endpoints
    ]


def _generate_colors(n: int) -> np.ndarray:
    """Generate *n* visually distinct RGB colors using HSV spacing.

    :param n: Number of colors.
    :type n: int
    :return: Array of shape ``(n, 3)`` with uint8 RGB values.
    :rtype: numpy.ndarray
    """
    colors = []
    for i in range(max(n, 1)):
        hue = i / max(n, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.9)
        colors.append([int(r * 255), int(g * 255), int(b * 255)])
    return np.array(colors, dtype=np.uint8)


# ============================================================================
# Main
# ============================================================================


def main() -> None:
    """Run the general Rerun visualization demo."""
    parser = argparse.ArgumentParser(
        description="General Rerun.io visualization demo with synthetic data."
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        metavar="PATH",
        help="Save recording to an .rrd file instead of spawning the viewer.",
    )
    parser.add_argument(
        "--connect",
        type=str,
        default=None,
        metavar="ADDR",
        help="Connect to a running Rerun Viewer at the given gRPC address"
        " (e.g. rerun+http://127.0.0.1:9876).",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Start a web-based Rerun Viewer (open in browser).  "
        "Ideal for WSL / headless environments.",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Initialize Rerun
    # ------------------------------------------------------------------
    rr.init("rerun_general_demo", spawn=False)

    if args.save:
        rr.save(args.save)
        print(f"Recording will be saved to: {args.save}")
    elif args.connect:
        rr.connect_grpc(args.connect)
        print(f"Streaming to Rerun Viewer at {args.connect}")
    elif args.serve:
        server_uri = rr.serve_grpc()
        rr.serve_web_viewer(open_browser=False, connect_to=server_uri)
        # URL-encode '+' and replace 127.0.0.1 for WSL/remote access
        from urllib.parse import quote  # noqa: C0415

        viewer_url = quote(server_uri.replace("127.0.0.1", "localhost"), safe="/:?")
        print(f"Rerun web viewer serving at http://localhost:9090/?url={viewer_url}")
    else:
        rr.spawn()
        print("Spawned Rerun Viewer.")

    # ------------------------------------------------------------------
    # 1. Log a synthetic grayscale image
    # ------------------------------------------------------------------
    width, height = 640, 480
    img = _make_gradient_image(width, height)
    rr.log("image/grayscale", rr.Image(img))
    rr.log("logs", rr.TextLog("Logged synthetic gradient image (640x480)"))

    # ------------------------------------------------------------------
    # 2. Log star-pattern line segments
    # ------------------------------------------------------------------
    star_lines = _make_star_lines(width / 2, height / 2, radius=150.0, n_rays=16)
    star_colors = _generate_colors(len(star_lines))
    rr.log(
        "image/star_lines",
        rr.LineStrips2D(star_lines, colors=star_colors, radii=1.5),
    )
    rr.log("logs", rr.TextLog(f"Logged {len(star_lines)} star rays"))

    # ------------------------------------------------------------------
    # 3. Log random segments with per-segment coloring
    # ------------------------------------------------------------------
    rng = np.random.default_rng(42)
    random_segs = _make_random_segments(width, height, n=40, rng=rng)
    seg_colors = _generate_colors(len(random_segs))
    rr.log(
        "image/random_segments",
        rr.LineStrips2D(random_segs, colors=seg_colors, radii=1.0),
    )
    rr.log("logs", rr.TextLog(f"Logged {len(random_segs)} random segments"))

    # ------------------------------------------------------------------
    # 4. Log endpoints as 2D points
    # ------------------------------------------------------------------
    all_pts = []
    for seg in random_segs:
        all_pts.append(seg[0])
        all_pts.append(seg[1])
    rr.log(
        "image/segment_endpoints",
        rr.Points2D(all_pts, colors=[255, 0, 0], radii=3.0),
    )

    # ------------------------------------------------------------------
    # 5. Demonstrate timelines — animate a rotating line
    # ------------------------------------------------------------------
    for frame in range(60):
        rr.set_time("frame", sequence=frame)
        angle = 2 * math.pi * frame / 60
        x_end = width / 2 + 120 * math.cos(angle)
        y_end = height / 2 + 120 * math.sin(angle)
        rr.log(
            "image/rotating_line",
            rr.LineStrips2D(
                [[[width / 2, height / 2], [x_end, y_end]]],
                colors=[[0, 255, 128]],
                radii=2.0,
            ),
        )

    print("\nDemo complete. Explore the Rerun Viewer!")
    print("  - Use the timeline scrubber to watch the rotating line.")
    print("  - Toggle entity visibility in the left panel.")
    print("  - Check the 'logs' entity for text messages.")

    if not args.save:
        # Keep the process alive briefly so the viewer can finish loading
        try:
            input("\nPress Enter to exit...")
        except (EOFError, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
