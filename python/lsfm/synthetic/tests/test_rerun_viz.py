"""Tests for Rerun visualization helpers.

These tests verify that logging functions run without errors on
representative data.  They do NOT verify visual output.
"""

from lsfm.synthetic.rerun_viz import reconstruction_blueprint


class TestReconstructionBlueprint:
    """Tests for blueprint factory."""

    def test_creates_blueprint(self) -> None:
        bp = reconstruction_blueprint(4)
        assert bp is not None

    def test_single_view(self) -> None:
        bp = reconstruction_blueprint(1)
        assert bp is not None

    def test_many_views_capped(self) -> None:
        # Should not crash with > 4 views (caps at 4 panels)
        bp = reconstruction_blueprint(10)
        assert bp is not None
