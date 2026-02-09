"""Integration tests for the le_eval Python extension module.

Tests cover:
  - Module import and core types
  - StringTable operations
  - GenericInputData and PerformanceResult
  - Task constants
  - ITask / Task interfaces
  - CVData and data providers
  - Performance measures
  - CVPerformanceTask subclassing from Python
  - CVPerformanceTest orchestrator
"""

from __future__ import annotations

import os
import tempfile

import numpy as np
import pytest

import le_eval


# ============================================================================
# Module-level tests
# ============================================================================


class TestModuleImport:
    """Test that the module and its core types are importable."""

    def test_core_types(self) -> None:
        assert hasattr(le_eval, "StringTable")
        assert hasattr(le_eval, "GenericInputData")
        assert hasattr(le_eval, "PerformanceResult")

    def test_task_types(self) -> None:
        assert hasattr(le_eval, "ITask")
        assert hasattr(le_eval, "Task")
        assert hasattr(le_eval, "CVPerformanceTask")

    def test_data_provider_types(self) -> None:
        assert hasattr(le_eval, "CVData")
        assert hasattr(le_eval, "FileCVDataProvider")
        assert hasattr(le_eval, "CVPerformanceData")
        assert hasattr(le_eval, "FileCVPerformanceDataProvider")
        assert hasattr(le_eval, "FileDataProvider")

    def test_measure_types(self) -> None:
        assert hasattr(le_eval, "Measure")
        assert hasattr(le_eval, "PerformanceMeasureBase")
        assert hasattr(le_eval, "CVPerformanceMeasure")

    def test_test_orchestrator(self) -> None:
        assert hasattr(le_eval, "CVPerformanceTest")

    def test_task_constants(self) -> None:
        assert le_eval.TASK_SQR == 1
        assert le_eval.TASK_RGB == 2
        assert le_eval.TASK_NO_3 == 4
        assert le_eval.TASK_NO_5 == 8


# ============================================================================
# StringTable tests
# ============================================================================


class TestStringTable:
    """Test StringTable class bindings."""

    def test_default_construction(self) -> None:
        table = le_eval.StringTable()
        assert table.rows == 0
        assert table.cols == 0
        assert table.size == 0

    def test_sized_construction(self) -> None:
        table = le_eval.StringTable(3, 4)
        assert table.rows == 3
        assert table.cols == 4
        assert table.size == 12

    def test_getitem_setitem(self) -> None:
        table = le_eval.StringTable(2, 3)
        table[0, 0] = "hello"
        table[1, 2] = "world"
        assert table[0, 0] == "hello"
        assert table[1, 2] == "world"

    def test_row(self) -> None:
        table = le_eval.StringTable(2, 3)
        table[0, 0] = "a"
        table[0, 1] = "b"
        table[0, 2] = "c"
        row = table.row(0)
        assert row == ["a", "b", "c"]

    def test_col(self) -> None:
        table = le_eval.StringTable(3, 2)
        table[0, 0] = "x"
        table[1, 0] = "y"
        table[2, 0] = "z"
        col = table.col(0)
        assert col == ["x", "y", "z"]

    def test_transpose(self) -> None:
        table = le_eval.StringTable(2, 3)
        table[0, 0] = "a"
        table[0, 1] = "b"
        table[1, 0] = "c"
        t = table.transpose()
        assert t.rows == 3
        assert t.cols == 2
        assert t[0, 0] == "a"
        assert t[1, 0] == "b"
        assert t[0, 1] == "c"

    def test_to_list(self) -> None:
        table = le_eval.StringTable(2, 2)
        table[0, 0] = "1"
        table[0, 1] = "2"
        table[1, 0] = "3"
        table[1, 1] = "4"
        lst = table.to_list()
        assert lst == [["1", "2"], ["3", "4"]]

    def test_save_csv(self) -> None:
        table = le_eval.StringTable(2, 2)
        table[0, 0] = "a"
        table[0, 1] = "b"
        table[1, 0] = "c"
        table[1, 1] = "d"
        with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as f:
            name = f.name
        try:
            table.save_csv(name)
            with open(name) as f:
                content = f.read()
            assert "a" in content
            assert "d" in content
        finally:
            os.unlink(name)

    def test_repr(self) -> None:
        table = le_eval.StringTable(1, 2)
        table[0, 0] = "x"
        table[0, 1] = "y"
        r = repr(table)
        assert "x" in r


# ============================================================================
# GenericInputData tests
# ============================================================================


class TestGenericInputData:
    """Test GenericInputData bindings."""

    def test_default(self) -> None:
        d = le_eval.GenericInputData()
        assert d.name == ""

    def test_named(self) -> None:
        d = le_eval.GenericInputData("test_data")
        assert d.name == "test_data"

    def test_name_readwrite(self) -> None:
        d = le_eval.GenericInputData()
        d.name = "modified"
        assert d.name == "modified"


# ============================================================================
# PerformanceResult tests
# ============================================================================


class TestPerformanceResult:
    """Test PerformanceResult bindings."""

    def test_default(self) -> None:
        r = le_eval.PerformanceResult()
        assert r.total == pytest.approx(0.0)
        assert r.mean == pytest.approx(0.0)
        assert r.stddev == pytest.approx(0.0)

    def test_readwrite(self) -> None:
        r = le_eval.PerformanceResult()
        r.total = 100.0
        r.mean = 10.0
        r.stddev = 2.5
        assert r.total == pytest.approx(100.0)
        assert r.mean == pytest.approx(10.0)
        assert r.stddev == pytest.approx(2.5)

    def test_repr(self) -> None:
        r = le_eval.PerformanceResult()
        r.total = 42.0
        s = repr(r)
        assert "42" in s
        assert "PerformanceResult" in s


# ============================================================================
# CVData tests
# ============================================================================


class TestCVData:
    """Test CVData bindings."""

    def test_default(self) -> None:
        d = le_eval.CVData()
        assert d.name == ""

    def test_name_access(self) -> None:
        d = le_eval.CVData()
        d.name = "image1"
        assert d.name == "image1"

    def test_src_access(self) -> None:
        d = le_eval.CVData()
        d.src = np.zeros((50, 50, 3), dtype=np.uint8)
        assert d.src.shape == (50, 50, 3)


# ============================================================================
# CVPerformanceData tests
# ============================================================================


class TestCVPerformanceData:
    """Test CVPerformanceData bindings."""

    def test_default(self) -> None:
        d = le_eval.CVPerformanceData()
        assert d.name == ""

    def test_construction_with_image(self) -> None:
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        d = le_eval.CVPerformanceData("test", img)
        assert d.name == "test"

    def test_inherits_generic_input_data(self) -> None:
        d = le_eval.CVPerformanceData()
        d.name = "inherited"
        assert d.name == "inherited"


# ============================================================================
# Measure tests
# ============================================================================


class TestMeasure:
    """Test Measure bindings."""

    def test_default(self) -> None:
        m = le_eval.Measure()
        assert m.source_name == ""
        assert m.task_name == ""

    def test_readwrite(self) -> None:
        m = le_eval.Measure()
        m.source_name = "src"
        m.task_name = "task"
        assert m.source_name == "src"
        assert m.task_name == "task"

    def test_clear(self) -> None:
        m = le_eval.Measure()
        m.source_name = "data"
        m.clear()


# ============================================================================
# PerformanceMeasureBase tests
# ============================================================================


class TestPerformanceMeasureBase:
    """Test PerformanceMeasureBase bindings."""

    def test_default(self) -> None:
        pm = le_eval.PerformanceMeasureBase()
        assert len(pm.durations) == 0

    def test_named(self) -> None:
        pm = le_eval.PerformanceMeasureBase("source", "task")
        assert pm.source_name == "source"
        assert pm.task_name == "task"

    def test_append_and_compute(self) -> None:
        pm = le_eval.PerformanceMeasureBase()
        pm.append_duration(1000000)
        pm.append_duration(2000000)
        pm.append_duration(3000000)
        result = pm.compute_result()
        assert result.total > 0
        assert result.mean > 0

    def test_clear(self) -> None:
        pm = le_eval.PerformanceMeasureBase()
        pm.append_duration(100)
        pm.clear()
        assert len(pm.durations) == 0


# ============================================================================
# CVPerformanceMeasure tests
# ============================================================================


class TestCVPerformanceMeasure:
    """Test CVPerformanceMeasure bindings."""

    def test_default(self) -> None:
        m = le_eval.CVPerformanceMeasure()
        assert m.width == pytest.approx(0.0)
        assert m.height == pytest.approx(0.0)

    def test_named(self) -> None:
        m = le_eval.CVPerformanceMeasure("src", "task")
        assert m.source_name == "src"

    def test_with_dimensions(self) -> None:
        m = le_eval.CVPerformanceMeasure("src", "task", 1920.0, 1080.0)
        assert m.width == pytest.approx(1920.0)
        assert m.height == pytest.approx(1080.0)

    def test_mega_pixels(self) -> None:
        m = le_eval.CVPerformanceMeasure("src", "task", 1000.0, 1000.0)
        assert pytest.approx(m.mega_pixels()) == 1.0

    def test_clear(self) -> None:
        m = le_eval.CVPerformanceMeasure("src", "task", 100.0, 100.0)
        m.clear()
        assert m.width == pytest.approx(0.0)


# ============================================================================
# Task tests (construction and Python subclassing)
# ============================================================================


class TestTask:
    """Test Task class — constructable and subclassable from Python."""

    def test_construction(self) -> None:
        """Task can be instantiated with a required name argument."""

        class MinimalTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

        task = MinimalTask("minimal")
        assert task.name == "minimal"
        assert not task.verbose

    def test_construction_verbose(self) -> None:
        """Task accepts an optional verbose flag."""

        class MinimalTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

        task = MinimalTask("verbose_task", True)
        assert task.verbose

    def test_is_itask(self) -> None:
        """Task instances satisfy the ITask interface."""

        class MinimalTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

        task = MinimalTask("t")
        assert isinstance(task, le_eval.ITask)

    def test_run_called(self) -> None:
        """Overridden run() is actually invoked."""
        call_count = 0

        class CountingTask(le_eval.Task):
            def run(self, loops: int) -> None:
                nonlocal call_count
                call_count += loops

        task = CountingTask("counter")
        task.run(5)
        assert call_count == 5

    def test_reset_override(self) -> None:
        """Optional reset() override is called from C++ interface."""
        reset_called = False

        class ResettableTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

            def reset(self) -> None:
                nonlocal reset_called
                reset_called = True

        task = ResettableTask("resettable")
        task.reset()
        assert reset_called

    def test_save_visual_results_override(self) -> None:
        """Optional save_visual_results() override is forwarded."""
        saved_path = None

        class SavingTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

            def save_visual_results(self, target_path: str) -> None:
                nonlocal saved_path
                saved_path = target_path

        task = SavingTask("saver")
        task.save_visual_results("/tmp/results")
        assert saved_path == "/tmp/results"

    def test_task_name_via_itask(self) -> None:
        """task_name() ITask method returns the name set at construction."""

        class MinimalTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

        task = MinimalTask("my_name")
        assert task.task_name() == "my_name"

    def test_name_readwrite(self) -> None:
        """name attribute is writable after construction."""

        class MinimalTask(le_eval.Task):
            def run(self, loops: int) -> None:
                pass

        task = MinimalTask("original")
        task.name = "updated"
        assert task.name == "updated"
        assert task.task_name() == "updated"


# ============================================================================
# CVPerformanceTask tests (Python subclassing)
# ============================================================================


class TestCVPerformanceTask:
    """Test CVPerformanceTask — subclassable from Python."""

    def test_construction(self) -> None:
        task = le_eval.CVPerformanceTask("test_task")
        assert task.name == "test_task"

    def test_flags(self) -> None:
        task = le_eval.CVPerformanceTask("test", le_eval.TASK_RGB | le_eval.TASK_SQR)
        assert task.rgb()
        assert task.sqr()

    def test_no_flags(self) -> None:
        task = le_eval.CVPerformanceTask("test", 0)
        assert not task.rgb()
        assert not task.sqr()

    def test_subclass(self) -> None:
        """Test that CVPerformanceTask can be subclassed in Python."""
        prepared = False
        ran = False

        class MyTask(le_eval.CVPerformanceTask):
            def prepare_impl(self, src: np.ndarray) -> None:
                nonlocal prepared
                prepared = True

            def run_impl(self, name: str, src: np.ndarray) -> None:
                nonlocal ran
                ran = True

        task = MyTask("my_task")
        assert task.name == "my_task"

    def test_border(self) -> None:
        task = le_eval.CVPerformanceTask("test", 0)
        b = task.border()
        assert isinstance(b, int)

    def test_verbose(self) -> None:
        task = le_eval.CVPerformanceTask("test", 0, True)
        assert task.verbose


# ============================================================================
# FileCVDataProvider tests
# ============================================================================


class TestFileCVDataProvider:
    """Test FileCVDataProvider bindings."""

    def test_construction(self) -> None:
        provider = le_eval.FileCVDataProvider("test_provider")
        assert provider.name == "test_provider"

    def test_rewind(self) -> None:
        provider = le_eval.FileCVDataProvider("test")
        provider.rewind()  # Should not raise

    def test_clear(self) -> None:
        provider = le_eval.FileCVDataProvider("test")
        provider.clear()  # Should not raise


# ============================================================================
# FileCVPerformanceDataProvider tests
# ============================================================================


class TestFileCVPerformanceDataProvider:
    """Test FileCVPerformanceDataProvider bindings."""

    def test_construction(self) -> None:
        provider = le_eval.FileCVPerformanceDataProvider("test_provider")
        assert provider.name == "test_provider"

    def test_rewind_clear(self) -> None:
        provider = le_eval.FileCVPerformanceDataProvider("test")
        provider.rewind()
        provider.clear()


# ============================================================================
# FileDataProvider tests
# ============================================================================


class TestFileDataProvider:
    """Test FileDataProvider convenience class."""

    def test_construction(self) -> None:
        # Use temp dir that exists
        provider = le_eval.FileDataProvider(tempfile.gettempdir(), "temp")
        assert provider.name == "temp"


# ============================================================================
# CVPerformanceTest orchestrator tests
# ============================================================================


class TestCVPerformanceTest:
    """Test CVPerformanceTest bindings."""

    def test_construction(self) -> None:
        providers: list = []
        test = le_eval.CVPerformanceTest(providers, "bench_test")
        assert test.show_mean

    def test_flags(self) -> None:
        test = le_eval.CVPerformanceTest([], "bench")
        test.show_total = True
        test.show_mean = True
        test.show_std_dev = False
        test.show_mega_pixel = True
        assert test.show_total
        assert not test.show_std_dev

    def test_result_table_empty(self) -> None:
        test = le_eval.CVPerformanceTest([], "bench")
        table = test.result_table()
        assert isinstance(table, le_eval.StringTable)

    def test_add_task(self) -> None:
        test = le_eval.CVPerformanceTest([], "bench")
        task = le_eval.CVPerformanceTask("dummy", 0)
        test.add_task(task)

    def test_clear(self) -> None:
        test = le_eval.CVPerformanceTest([], "bench")
        test.clear()


# ============================================================================
# accumulate_measures tests
# ============================================================================


class TestAccumulateMeasures:
    """Test the accumulate_measures free function."""

    def test_empty(self) -> None:
        result = le_eval.accumulate_measures([])
        assert result.mega_pixels() == pytest.approx(0.0)

    def test_single(self) -> None:
        m = le_eval.CVPerformanceMeasure("src", "task", 100.0, 100.0)
        m.append_duration(1000)
        result = le_eval.accumulate_measures([m])
        assert len(result.durations) >= 1
