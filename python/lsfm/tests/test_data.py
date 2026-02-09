"""Tests for lsfm.data — dataset path resolution."""

from __future__ import annotations

from pathlib import Path

import pytest

from lsfm.data import TestImages, _is_image


# =============================================================================
# Helper function tests
# =============================================================================


class TestIsImage:
    """Verify the image-extension check helper."""

    @pytest.mark.parametrize(
        "name",
        [
            "photo.png",
            "photo.jpg",
            "photo.jpeg",
            "photo.bmp",
            "photo.tif",
            "photo.tiff",
        ],
    )
    def test_valid_extensions(self, name: str) -> None:
        assert _is_image(Path(name))

    @pytest.mark.parametrize(
        "name",
        ["photo.PNG", "photo.JPG", "photo.Jpeg", "photo.TIFF"],
    )
    def test_case_insensitive(self, name: str) -> None:
        assert _is_image(Path(name))

    @pytest.mark.parametrize(
        "name",
        ["readme.txt", "data.csv", "model.onnx", "script.py", "archive.tar.gz"],
    )
    def test_non_image_extensions(self, name: str) -> None:
        assert not _is_image(Path(name))


# =============================================================================
# TestImages — path resolution
# =============================================================================


class TestTestImages:
    """Test the TestImages resolver."""

    @pytest.fixture()
    def images(self) -> TestImages:
        """Create a TestImages instance."""
        return TestImages()

    def test_search_paths_non_empty(self, images: TestImages) -> None:
        """At least one search path must be detected."""
        assert len(images.search_paths) > 0

    def test_search_paths_are_directories(self, images: TestImages) -> None:
        for p in images.search_paths:
            assert p.is_dir(), f"Search path is not a directory: {p}"

    # --- windmill.jpg ---

    def test_windmill_exists(self, images: TestImages) -> None:
        path = images.windmill()
        assert path.exists()
        assert path.name == "windmill.jpg"

    def test_windmill_is_absolute(self, images: TestImages) -> None:
        assert images.windmill().is_absolute()

    # --- generic get() ---

    def test_get_windmill(self, images: TestImages) -> None:
        path = images.get("windmill.jpg")
        assert path.exists()

    def test_get_not_found_raises(self, images: TestImages) -> None:
        with pytest.raises(FileNotFoundError, match="nonexistent_image_12345.png"):
            images.get("nonexistent_image_12345.png")

    # --- noise dataset ---

    def test_noise_image_exists(self, images: TestImages) -> None:
        path = images.noise("bike.png")
        assert path.exists()
        assert path.name == "bike.png"

    def test_noise_images_iterator(self, images: TestImages) -> None:
        noise_list = list(images.noise_images())
        assert len(noise_list) > 0
        for p in noise_list:
            assert _is_image(p)
            assert p.exists()

    def test_noise_images_sorted(self, images: TestImages) -> None:
        noise_list = list(images.noise_images())
        names = [p.name for p in noise_list]
        assert names == sorted(names)

    # --- BSDS500 ---

    def test_bsds500_single_image(self, images: TestImages) -> None:
        try:
            path = images.bsds500("100007.jpg")
        except FileNotFoundError:
            pytest.skip("BSDS500 dataset not available")
        assert path.exists()

    def test_bsds500_iterator(self, images: TestImages) -> None:
        all_images = list(images.bsds500())
        if not all_images:
            pytest.skip("BSDS500 dataset not available")
        for p in all_images:
            assert _is_image(p)

    # --- MDB stereo ---

    def test_stereo_scenes_returns_names(self, images: TestImages) -> None:
        scenes = list(images.stereo_scenes(resolution="Q"))
        # MDB-Q should have some scenes if setup was run
        if scenes:
            assert all(isinstance(s, str) for s in scenes)
            assert "Adirondack" in scenes

    def test_stereo_pair_quarter(self, images: TestImages) -> None:
        scenes = list(images.stereo_scenes(resolution="Q"))
        if not scenes:
            pytest.skip("MDB-Q dataset not available")
        left, right = images.stereo_pair(scenes[0], resolution="Q")
        assert left.exists()

    # --- is_bazel_run ---

    def test_is_bazel_run_type(self) -> None:
        assert isinstance(TestImages.is_bazel_run(), bool)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
