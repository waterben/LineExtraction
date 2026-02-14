"""Dataset and test image path resolution for Bazel and standalone builds.

Mirrors the C++ ``TestImages`` utility (``libs/utility/include/utility/test_images.hpp``)
and provides Pythonic access to the project's image datasets (windmill, BSDS500,
MDB stereo pairs, noise images, York Urban, Wireframe).

Typical usage::

    from lsfm.data import TestImages

    images = TestImages()
    img = cv2.imread(str(images.windmill()))

    # Iterate all BSDS500 images
    for path in images.bsds500():
        print(path)

    # Load a stereo pair
    left, right = images.stereo_pair("Adirondack", resolution="Q")
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterator


# Supported image file extensions (lowercase, with dot).
_IMAGE_EXTENSIONS: frozenset[str] = frozenset(
    {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
)


def _is_image(path: Path) -> bool:
    """Check whether *path* has a supported image extension.

    :param path: File path to check.
    :type path: Path
    :return: ``True`` if the suffix is a recognised image format.
    :rtype: bool
    """
    return path.suffix.lower() in _IMAGE_EXTENSIONS


class TestImages:
    """Resolve test image paths across Bazel runfiles and standalone layouts.

    The resolver searches in the following order:

    1. **Bazel runfiles** — when ``RUNFILES_DIR`` is set (``bazel test`` / ``bazel run``).
    2. **Workspace-relative paths** — ``resources/``, ``resources/datasets/``,
       and several ``../`` variants so it works from ``build/bin/`` as well.

    :param argv0: Optional ``sys.argv[0]`` for executable-relative resolution.
    :type argv0: str or None

    .. note::
       Name starts with ``Test`` for parity with C++ ``TestImages``.
       The class is intentionally **not** a singleton; you can create several
       instances with different *argv0* values in tests.

    Example::

        images = TestImages()
        path = images.windmill()
        assert path.exists()
    """

    __test__ = False  # prevent pytest collection

    def __init__(self, argv0: str | None = None) -> None:
        self._argv0: str = argv0 or ""
        self._search_paths: list[Path] = []
        self._detect_search_paths()

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------

    def windmill(self) -> Path:
        """Return the path to ``windmill.jpg``.

        :return: Resolved path to the windmill test image.
        :rtype: Path
        """
        return self.get("windmill.jpg")

    def noise(self, name: str) -> Path:
        """Return the path to an image in the *noise* dataset.

        :param name: Image filename, e.g. ``"bike.png"``.
        :type name: str
        :return: Resolved path.
        :rtype: Path
        """
        return self.get(f"noise/{name}")

    def noise_images(self) -> Iterator[Path]:
        """Yield all images in the *noise* dataset directory.

        :return: Iterator of resolved image paths.
        :rtype: Iterator[Path]
        """
        yield from self._iter_images("noise")

    def bsds500(
        self, name: str | None = None, split: str = ""
    ) -> Path | Iterator[Path]:
        """Access BSDS500 images.

        When *name* is given, return the single resolved path.
        When *name* is ``None``, return an iterator over **all** images
        (optionally filtered by *split*).

        :param name: Image filename, e.g. ``"100007.jpg"``.  ``None`` to iterate.
        :type name: str or None
        :param split: Dataset split — ``"train"``, ``"test"``, ``"val"``, or ``""``
            for the flat directory.
        :type split: str
        :return: Resolved path or iterator of paths.
        :rtype: Path or Iterator[Path]
        """
        if name is not None:
            subpath = f"BSDS500/{split}/{name}" if split else f"BSDS500/{name}"
            return self.get(subpath)
        subdir = f"BSDS500/{split}" if split else "BSDS500"
        return self._iter_images(subdir)

    def york_urban(self, name: str | None = None) -> Path | Iterator[Path]:
        """Access York Urban Line Segment Database images.

        When *name* is given, return the single resolved path.
        When *name* is ``None``, return an iterator over **all** images.

        :param name: Image filename, e.g. ``"P1010001.jpg"``.  ``None`` to iterate.
        :type name: str or None
        :return: Resolved path or iterator of paths.
        :rtype: Path or Iterator[Path]

        .. note::
           Requires the dataset to be downloaded first:
           ``./tools/scripts/setup_york_urban.sh``
        """
        if name is not None:
            return self.get(f"YorkUrban/images/{name}")
        return self._iter_images("YorkUrban/images")

    def wireframe(self, name: str | None = None) -> Path | Iterator[Path]:
        """Access Wireframe dataset images (Huang et al., CVPR 2018).

        When *name* is given, return the single resolved path.
        When *name* is ``None``, return an iterator over **all** images.

        :param name: Image filename, e.g. ``"00031546.jpg"``.  ``None`` to iterate.
        :type name: str or None
        :return: Resolved path or iterator of paths.
        :rtype: Path or Iterator[Path]

        .. note::
           Requires the dataset to be downloaded first:
           ``./tools/scripts/setup_wireframe.sh``
        """
        if name is not None:
            return self.get(f"Wireframe/images/{name}")
        return self._iter_images("Wireframe/images")

    def ground_truth_csv(self, name: str) -> Path:
        """Return the path to a ground truth CSV file.

        :param name: CSV filename, e.g. ``"york_urban_gt.csv"``
            or ``"wireframe_gt.csv"``.
        :type name: str
        :return: Resolved path to the CSV file.
        :rtype: Path
        """
        return self.get(f"ground_truth/{name}")

    def stereo_pair(self, scene: str, resolution: str = "H") -> tuple[Path, Path]:
        """Return the left/right image paths for an MDB stereo scene.

        :param scene: Scene name, e.g. ``"Adirondack"``.
        :type scene: str
        :param resolution: ``"Q"`` (quarter), ``"H"`` (half), ``"F"`` (full).
        :type resolution: str
        :return: Tuple of ``(left, right)`` image paths.
        :rtype: tuple[Path, Path]
        :raises FileNotFoundError: If neither left nor right image can be found.
        """
        res_dir = f"MDB/MiddEval3-{resolution}"
        left = self._resolve_any(
            f"{res_dir}/{scene}/im0.png",
            f"{res_dir}/{scene}.png",
        )
        right = self._resolve_any(
            f"{res_dir}/{scene}/im1.png",
        )
        return left, right

    def stereo_scenes(self, resolution: str = "H") -> Iterator[str]:
        """Yield available MDB scene names for a given resolution.

        :param resolution: ``"Q"``, ``"H"``, or ``"F"``.
        :type resolution: str
        :return: Iterator of scene name strings.
        :rtype: Iterator[str]
        """
        res_dir = f"MDB/MiddEval3-{resolution}"
        for base in self._search_paths:
            d = base / res_dir
            if d.is_dir():
                for child in sorted(d.iterdir()):
                    if child.is_file() and _is_image(child):
                        yield child.stem
                    elif child.is_dir():
                        yield child.name
                return

    # ------------------------------------------------------------------
    # Generic resolver
    # ------------------------------------------------------------------

    def get(self, relative_path: str) -> Path:
        """Resolve *relative_path* against the known search paths.

        :param relative_path: Path relative to a dataset root,
            e.g. ``"windmill.jpg"`` or ``"BSDS500/100007.jpg"``.
        :type relative_path: str
        :return: Absolute path to the file.
        :rtype: Path
        :raises FileNotFoundError: If the file cannot be found in any search path.
        """
        for base in self._search_paths:
            candidate = base / relative_path
            if candidate.exists():
                return candidate.resolve()

        searched = "\n  ".join(str(p) for p in self._search_paths)
        raise FileNotFoundError(
            f"Test image not found: {relative_path}\nSearched in:\n  {searched}"
        )

    @property
    def search_paths(self) -> list[Path]:
        """Return the list of directories being searched (read-only copy).

        :return: List of search path directories.
        :rtype: list[Path]
        """
        return list(self._search_paths)

    @staticmethod
    def is_bazel_run() -> bool:
        """Check whether the process is running under ``bazel test`` / ``bazel run``.

        :return: ``True`` when Bazel runfiles environment variables are set.
        :rtype: bool
        """
        return (
            os.environ.get("RUNFILES_DIR") is not None
            or os.environ.get("RUNFILES_MANIFEST_FILE") is not None
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _detect_search_paths(self) -> None:
        """Populate ``_search_paths`` from runfiles and filesystem probing."""
        self._search_paths.clear()

        # 1. Bazel runfiles
        if self.is_bazel_run():
            runfiles_dir = os.environ.get("RUNFILES_DIR", "")
            if runfiles_dir:
                rd = Path(runfiles_dir)
                # External BSDS500 repository images
                self._add_if_exists(rd / "bsds500" / "BSDS500" / "data" / "images")
                # Local resources (windmill.jpg, etc.)
                self._add_if_exists(rd / "line_extraction" / "resources")
                # Local datasets (noise, MDB, YorkUrban, Wireframe, ground_truth)
                self._add_if_exists(rd / "line_extraction" / "resources" / "datasets")
            else:
                # Manifest-only mode (common on Windows): parse
                # RUNFILES_MANIFEST_FILE to resolve runfile paths.
                self._add_manifest_paths()

        # 2. Relative to cwd (works for CMake and manual runs)
        relative_paths = [
            "resources",
            "resources/datasets",
            "../resources",
            "../resources/datasets",
            "../../resources",
            "../../resources/datasets",
            "../../../resources",
            "../../../resources/datasets",
        ]
        for rel in relative_paths:
            self._add_if_exists(Path(rel))

        # 3. Relative to executable (if argv0 given)
        if self._argv0:
            exe_dir = Path(self._argv0).parent
            if exe_dir != Path("."):
                for rel in relative_paths:
                    self._add_if_exists(exe_dir / rel)

    def _add_manifest_paths(self) -> None:
        """Derive search paths from ``RUNFILES_MANIFEST_FILE``.

        The manifest is a plain-text file where each line maps a runfiles-
        relative path to an absolute filesystem path, separated by a space::

            line_extraction/resources/windmill.jpg /abs/path/to/windmill.jpg
            bsds500/BSDS500/data/images/test/100007.jpg /abs/...

        We scan for known directory prefixes and add the corresponding
        filesystem directories to :pyattr:`_search_paths`.

        :meta private:
        """
        manifest = os.environ.get("RUNFILES_MANIFEST_FILE", "")
        if not manifest or not Path(manifest).is_file():
            return

        # Runfile prefixes we care about → number of segments to strip from
        # the filesystem path to get the directory root.
        _PREFIXES: list[tuple[str, int]] = [
            ("bsds500/BSDS500/data/images/", 0),
            ("line_extraction/resources/datasets/", 0),
            ("line_extraction/resources/", 0),
        ]

        found: set[Path] = set()
        try:
            with open(manifest, encoding="utf-8") as fp:
                for raw_line in fp:
                    parts = raw_line.rstrip("\n").split(" ", 1)
                    if len(parts) != 2:
                        continue
                    runpath, fspath = parts
                    for prefix, _ in _PREFIXES:
                        if runpath.startswith(prefix):
                            # Compute the filesystem directory that corresponds
                            # to `prefix` by stripping the suffix from fspath.
                            suffix = runpath[len(prefix) :]
                            if suffix:
                                base = Path(fspath[: -len(suffix)]).resolve()
                            else:
                                base = Path(fspath).resolve()
                            if base.is_dir() and base not in found:
                                found.add(base)
                                self._search_paths.append(base)
                            break  # first matching prefix wins
        except OSError:
            pass

    def _add_if_exists(self, path: Path) -> None:
        """Append *path* to search paths if it is an existing directory.

        :param path: Directory to check and potentially add.
        :type path: Path
        """
        resolved = path.resolve()
        if resolved.is_dir() and resolved not in self._search_paths:
            self._search_paths.append(resolved)

    def _iter_images(self, subdir: str) -> Iterator[Path]:
        """Yield all image files under *subdir* in the first matching search path.

        :param subdir: Subdirectory relative to a dataset root.
        :type subdir: str
        :return: Iterator of resolved image paths, sorted alphabetically.
        :rtype: Iterator[Path]
        """
        for base in self._search_paths:
            d = base / subdir
            if d.is_dir():
                yield from sorted(
                    p for p in d.iterdir() if p.is_file() and _is_image(p)
                )
                return

    def _resolve_any(self, *relative_paths: str) -> Path:
        """Try multiple relative paths and return the first match.

        :param relative_paths: One or more relative paths to try.
        :type relative_paths: str
        :return: The first successfully resolved path.
        :rtype: Path
        :raises FileNotFoundError: If none of the paths resolve.
        """
        for rel in relative_paths:
            for base in self._search_paths:
                candidate = base / rel
                if candidate.exists():
                    return candidate.resolve()

        searched = "\n  ".join(str(p) for p in self._search_paths)
        tried = ", ".join(relative_paths)
        raise FileNotFoundError(f"None of [{tried}] found.\nSearched in:\n  {searched}")
