#!/usr/bin/env python3
"""Convert external dataset annotations to GroundTruthLoader CSV format.

This script converts ground truth line segment annotations from York Urban
(.mat) and Wireframe (.pkl/.json) datasets into the CSV format expected by
``le_algorithm.GroundTruthLoader``.

CSV format::

    image_name,x1,y1,x2,y2
    image001.jpg,10.0,20.0,100.0,200.0
    ...

Usage::

    # York Urban (.mat files)
    python convert_ground_truth.py --dataset york_urban \\
        --input-dir resources/datasets/YorkUrban/annotations \\
        --image-dir resources/datasets/YorkUrban/images \\
        --output resources/datasets/ground_truth/york_urban_gt.csv

    # Wireframe (.pkl files)
    python convert_ground_truth.py --dataset wireframe \\
        --input-dir resources/datasets/Wireframe/annotations \\
        --image-dir resources/datasets/Wireframe/images \\
        --output resources/datasets/ground_truth/wireframe_gt.csv
"""

from __future__ import annotations

import argparse
import csv
import pickle
import sys
from pathlib import Path
from typing import NamedTuple


class LineSegmentGT(NamedTuple):
    """A ground truth line segment with image association.

    :param image_name: Filename of the source image.
    :type image_name: str
    :param x1: X coordinate of start point.
    :type x1: float
    :param y1: Y coordinate of start point.
    :type y1: float
    :param x2: X coordinate of end point.
    :type x2: float
    :param y2: Y coordinate of end point.
    :type y2: float
    """

    image_name: str
    x1: float
    y1: float
    x2: float
    y2: float


def convert_york_urban(
    input_dir: Path,
    image_dir: Path,
) -> list[LineSegmentGT]:
    """Convert York Urban .mat annotations to line segments.

    The York Urban DB stores annotations as MATLAB ``.mat`` files, one per
    image, named ``<ImageStem>LinesAndVP.mat``.  Each file contains:

    - ``lines`` — shape ``(2N, 2)`` holding pairs of ``(x, y)`` endpoint
      coordinates: rows ``[2i, 2i+1]`` form the start/end of segment *i*.
    - ``vp_association`` — vanishing-point association (not used here).

    :param input_dir: Directory containing ``*LinesAndVP.mat`` files.
    :type input_dir: Path
    :param image_dir: Directory containing the corresponding images.
    :type image_dir: Path
    :return: List of ground truth line segments.
    :rtype: list[LineSegmentGT]
    :raises ImportError: If scipy is not installed.
    """
    try:
        import scipy.io  # noqa: F401
    except ImportError:
        print("ERROR: scipy is required for loading .mat files.", file=sys.stderr)
        print("Install with: pip install scipy", file=sys.stderr)
        sys.exit(1)

    import numpy as np

    segments: list[LineSegmentGT] = []

    # Only process *LinesAndVP.mat files (the ones with line segment data).
    # Other .mat files (CamParams, Orthogonal_CamParams, etc.) are skipped.
    mat_files = sorted(input_dir.glob("*LinesAndVP.mat"))

    if not mat_files:
        # Fall back to all .mat files if naming convention differs
        mat_files = sorted(input_dir.glob("*.mat"))

    if not mat_files:
        print(f"WARNING: No .mat files found in {input_dir}", file=sys.stderr)
        return segments

    # Build a mapping from annotation stem to image filename
    image_map = _build_image_map(image_dir)

    for mat_path in mat_files:
        # Extract the image stem from the annotation filename.
        # e.g. "P1020171LinesAndVP.mat" -> "P1020171"
        stem = mat_path.stem
        for suffix in (
            "LinesAndVP",
            "GroundTruthVP_CamParams",
            "GroundTruthVP_Orthogonal_CamParams",
        ):
            if stem.endswith(suffix):
                stem = stem[: -len(suffix)]
                break

        image_name = image_map.get(stem)
        if image_name is None:
            image_name = _find_image_for_stem(stem, image_dir)
            if image_name is None:
                print(
                    f"WARNING: No image found for annotation {mat_path.name}",
                    file=sys.stderr,
                )
                continue

        try:
            data = scipy.io.loadmat(str(mat_path))
        except Exception as exc:
            print(f"WARNING: Failed to load {mat_path.name}: {exc}", file=sys.stderr)
            continue

        # Try the standard 'lines' key first
        lines_data = data.get("lines")

        if lines_data is not None:
            arr = np.asarray(lines_data, dtype=float)

            if arr.ndim == 2 and arr.shape[1] == 2 and arr.shape[0] % 2 == 0:
                # York Urban format: (2N, 2) — pairs of (x, y) endpoint rows
                arr = arr.reshape(-1, 4)  # -> (N, 4): [x1, y1, x2, y2]
            elif arr.ndim == 2 and arr.shape[1] == 4:
                pass  # Already (N, 4)
            else:
                print(
                    f"WARNING: Unexpected 'lines' shape {arr.shape} in {mat_path.name}",
                    file=sys.stderr,
                )
                continue

            for row in arr:
                segments.append(
                    LineSegmentGT(
                        image_name=image_name,
                        x1=float(row[0]),
                        y1=float(row[1]),
                        x2=float(row[2]),
                        y2=float(row[3]),
                    )
                )
            continue

        # Fallback: search for any suitable numeric array
        for key in ("segments", "line_segments", "segs", "groundTruth"):
            if key in data:
                lines_data = data[key]
                break

        if lines_data is None:
            for key, val in data.items():
                if key.startswith("_"):
                    continue
                try:
                    arr = np.asarray(val, dtype=float)
                    if arr.ndim == 2 and arr.shape[1] == 4:
                        lines_data = arr
                        break
                    elif arr.ndim == 2 and arr.shape[1] == 2 and arr.shape[0] % 2 == 0:
                        lines_data = arr.reshape(-1, 4)
                        break
                except (ValueError, TypeError):
                    continue

        if lines_data is None:
            print(
                f"WARNING: No line segment data found in {mat_path.name}. "
                f"Available keys: {[k for k in data if not k.startswith('_')]}",
                file=sys.stderr,
            )
            continue

        lines_array = np.asarray(lines_data, dtype=float)
        if lines_array.ndim != 2 or lines_array.shape[1] < 4:
            print(
                f"WARNING: Unexpected shape {lines_array.shape} in {mat_path.name}",
                file=sys.stderr,
            )
            continue

        for row in lines_array:
            segments.append(
                LineSegmentGT(
                    image_name=image_name,
                    x1=float(row[0]),
                    y1=float(row[1]),
                    x2=float(row[2]),
                    y2=float(row[3]),
                )
            )

    print(f"York Urban: {len(segments)} segments from {len(mat_files)} annotations")
    return segments


def convert_wireframe(
    input_dir: Path,
    image_dir: Path,
) -> list[LineSegmentGT]:
    """Convert Wireframe .pkl annotations to line segments.

    The Wireframe dataset stores annotations as pickle files with:
    - ``points``: list of (x, y) junction coordinates
    - ``lines``: list of (idx1, idx2) index pairs into the points array
    - ``imagename``: the source image filename

    Line segments are reconstructed by looking up endpoint coordinates
    from the junction list.

    :param input_dir: Directory containing .pkl annotation files.
    :type input_dir: Path
    :param image_dir: Directory containing the corresponding images.
    :type image_dir: Path
    :return: List of ground truth line segments.
    :rtype: list[LineSegmentGT]
    """
    segments: list[LineSegmentGT] = []
    pkl_files = sorted(input_dir.glob("*.pkl"))

    if not pkl_files:
        # Also try .json files (alternative annotation format)
        json_segments = _convert_wireframe_json(input_dir, image_dir)
        if json_segments:
            return json_segments
        print(f"WARNING: No .pkl or .json files found in {input_dir}", file=sys.stderr)
        return segments

    image_map = _build_image_map(image_dir)

    for pkl_path in pkl_files:
        try:
            with open(pkl_path, "rb") as f:
                data = pickle.load(f, encoding="latin1")
        except Exception as exc:
            print(f"WARNING: Failed to load {pkl_path.name}: {exc}", file=sys.stderr)
            continue

        # Get image name from annotation or filename
        image_name = data.get("imagename", data.get("filename", ""))
        if not image_name:
            stem = pkl_path.stem
            image_name = image_map.get(stem, f"{stem}.jpg")

        # Ensure image_name is just the filename (not full path)
        image_name = Path(image_name).name

        # Extract points and lines
        points = data.get("points", [])
        lines = data.get("lines", [])

        if not points or not lines:
            # Try alternative keys
            points = data.get("junctions", data.get("pts", []))
            lines = data.get("edges", data.get("line_set", []))

        if not points or not lines:
            print(
                f"WARNING: No points/lines in {pkl_path.name}. "
                f"Keys: {list(data.keys())}",
                file=sys.stderr,
            )
            continue

        for idx1, idx2 in lines:
            try:
                x1, y1 = float(points[idx1][0]), float(points[idx1][1])
                x2, y2 = float(points[idx2][0]), float(points[idx2][1])
                segments.append(
                    LineSegmentGT(
                        image_name=image_name,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                    )
                )
            except (IndexError, TypeError, ValueError) as exc:
                print(
                    f"WARNING: Invalid line in {pkl_path.name}: {exc}",
                    file=sys.stderr,
                )
                continue

    print(f"Wireframe: {len(segments)} segments from {len(pkl_files)} annotations")
    return segments


def _convert_wireframe_json(
    input_dir: Path,
    image_dir: Path,
) -> list[LineSegmentGT]:
    """Convert Wireframe JSON annotations (alternative format).

    Supports two JSON formats:

    1. **LCNN batch format** (``train.json`` / ``valid.json``): a single JSON
       file containing a list of entries, each with ``"filename"`` and
       ``"lines"`` (a flat array reshaped to Nx2x2 giving [[x1,y1],[x2,y2]]).
    2. **Per-image format**: individual JSON files, one per image, with either
       ``{"lines": [[x1,y1,x2,y2], ...]}`` or ``{"junctions": ..., "edges": ...}``.

    :param input_dir: Directory containing .json annotation files.
    :type input_dir: Path
    :param image_dir: Directory containing the corresponding images.
    :type image_dir: Path
    :return: List of ground truth line segments.
    :rtype: list[LineSegmentGT]
    """
    import json

    import numpy as np

    segments: list[LineSegmentGT] = []
    json_files = sorted(input_dir.glob("*.json"))

    if not json_files:
        return segments

    for json_path in json_files:
        try:
            with open(json_path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            print(f"WARNING: Failed to load {json_path.name}: {exc}", file=sys.stderr)
            continue

        # ---- LCNN batch format: list of annotation dicts ----
        if isinstance(data, list):
            for entry in data:
                image_name = Path(
                    entry.get("filename", entry.get("imagename", ""))
                ).name
                if not image_name:
                    continue
                raw_lines = entry.get("lines", [])
                if not raw_lines:
                    continue
                # Lines are stored flat: [x1,y1,x2,y2,...] → reshape to (N,2,2)
                lines_arr = np.array(raw_lines, dtype=float).reshape(-1, 2, 2)
                for line in lines_arr:
                    segments.append(
                        LineSegmentGT(
                            image_name=image_name,
                            x1=float(line[0, 0]),
                            y1=float(line[0, 1]),
                            x2=float(line[1, 0]),
                            y2=float(line[1, 1]),
                        )
                    )
            print(
                f"Wireframe (batch JSON): {json_path.name} — "
                f"{len(segments)} segments so far"
            )
            continue

        # ---- Per-image format: single dict per file ----

        image_name = data.get(
            "filename", data.get("imagename", json_path.stem + ".jpg")
        )
        image_name = Path(image_name).name

        # Try direct line format: {"lines": [[x1,y1,x2,y2], ...]}
        lines = data.get("lines", [])
        if lines and isinstance(lines[0], (list, tuple)) and len(lines[0]) == 4:
            for line in lines:
                segments.append(
                    LineSegmentGT(
                        image_name=image_name,
                        x1=float(line[0]),
                        y1=float(line[1]),
                        x2=float(line[2]),
                        y2=float(line[3]),
                    )
                )
        elif lines:
            # LCNN-style per-image: flat list → (N, 2, 2)
            try:
                lines_arr = np.array(lines, dtype=float).reshape(-1, 2, 2)
                for line in lines_arr:
                    segments.append(
                        LineSegmentGT(
                            image_name=image_name,
                            x1=float(line[0, 0]),
                            y1=float(line[0, 1]),
                            x2=float(line[1, 0]),
                            y2=float(line[1, 1]),
                        )
                    )
            except (ValueError, IndexError) as exc:
                print(
                    f"WARNING: Cannot parse lines in {json_path.name}: {exc}",
                    file=sys.stderr,
                )
        elif "junctions" in data and "edges" in data:
            # Junction + edge format
            junctions = data["junctions"]
            edges = data["edges"]
            for idx1, idx2 in edges:
                x1, y1 = float(junctions[idx1][0]), float(junctions[idx1][1])
                x2, y2 = float(junctions[idx2][0]), float(junctions[idx2][1])
                segments.append(
                    LineSegmentGT(
                        image_name=image_name,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                    )
                )

    print(f"Wireframe (JSON): {len(segments)} segments from {len(json_files)} files")
    return segments


def _build_image_map(image_dir: Path) -> dict[str, str]:
    """Build a mapping from image stem to full filename.

    :param image_dir: Directory containing image files.
    :type image_dir: Path
    :return: Dictionary mapping stem (without extension) to full filename.
    :rtype: dict[str, str]
    """
    result: dict[str, str] = {}
    if not image_dir.is_dir():
        return result
    for img_path in image_dir.iterdir():
        if img_path.suffix.lower() in {
            ".jpg",
            ".jpeg",
            ".png",
            ".bmp",
            ".tif",
            ".tiff",
        }:
            result[img_path.stem] = img_path.name
    return result


def _find_image_for_stem(stem: str, image_dir: Path) -> str | None:
    """Find an image file matching the given stem.

    :param stem: Filename stem (without extension) to search for.
    :type stem: str
    :param image_dir: Directory to search in.
    :type image_dir: Path
    :return: Image filename if found, None otherwise.
    :rtype: str or None
    """
    for ext in (".jpg", ".jpeg", ".png", ".bmp"):
        candidate = image_dir / f"{stem}{ext}"
        if candidate.exists():
            return candidate.name
    return None


def write_csv(
    segments: list[LineSegmentGT],
    output_path: Path,
) -> None:
    """Write ground truth segments to CSV file.

    :param segments: List of ground truth line segments.
    :type segments: list[LineSegmentGT]
    :param output_path: Path to write the CSV file.
    :type output_path: Path
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["image_name", "x1", "y1", "x2", "y2"])
        for seg in segments:
            writer.writerow(
                [
                    seg.image_name,
                    f"{seg.x1:.2f}",
                    f"{seg.y1:.2f}",
                    f"{seg.x2:.2f}",
                    f"{seg.y2:.2f}",
                ]
            )

    # Count unique images
    unique_images = len({seg.image_name for seg in segments})
    print(f"Written {len(segments)} segments ({unique_images} images) to {output_path}")


def main() -> None:
    """Run the ground truth conversion.

    Parses command-line arguments and dispatches to the appropriate
    converter function based on the ``--dataset`` flag.
    """
    parser = argparse.ArgumentParser(
        description="Convert dataset annotations to GroundTruthLoader CSV format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--dataset",
        required=True,
        choices=["york_urban", "wireframe"],
        help="Dataset type to convert",
    )
    parser.add_argument(
        "--input-dir",
        required=True,
        type=Path,
        help="Directory containing annotation files (.mat or .pkl/.json)",
    )
    parser.add_argument(
        "--image-dir",
        required=True,
        type=Path,
        help="Directory containing the corresponding images",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output CSV file path",
    )

    args = parser.parse_args()

    if not args.input_dir.is_dir():
        print(f"ERROR: Input directory not found: {args.input_dir}", file=sys.stderr)
        sys.exit(1)

    if args.dataset == "york_urban":
        segments = convert_york_urban(args.input_dir, args.image_dir)
    elif args.dataset == "wireframe":
        segments = convert_wireframe(args.input_dir, args.image_dir)
    else:
        print(f"ERROR: Unknown dataset: {args.dataset}", file=sys.stderr)
        sys.exit(1)

    if not segments:
        print(
            "WARNING: No segments extracted. Check input directory and format.",
            file=sys.stderr,
        )
        sys.exit(1)

    write_csv(segments, args.output)


if __name__ == "__main__":
    main()
