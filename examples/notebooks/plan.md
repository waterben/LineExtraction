# Plan: Multi-View Reconstruction Demo with LIMAP

## TL;DR

Rename current `demo_3d_reconstruction.ipynb` → `demo_stereo_reconstruction_limap.ipynb` (it's a stereo demo using LIMAP pairwise triangulation). Create new `demo_multiview_reconstruction.ipynb` that showcases LIMAP's **full multi-view pipeline** (`line_triangulation` runner) on ETH3D courtyard scene (38 images), with our LSD detector integrated via `BaseDetector` subclass, comparing against LIMAP's built-in LSD.

## Phase 1: Rename Existing Demo

1. Rename `examples/notebooks/demo_3d_reconstruction.ipynb` → `examples/notebooks/demo_stereo_reconstruction_limap.ipynb`
2. Update title/description in the markdown cells to reflect it's a stereo (2-view) LIMAP demo
3. Update any cross-references in other notebooks or README files

## Phase 2: Create LIMAP BaseDetector Adapter

4. In `python/lsfm/limap_compat.py`, create `LsfmLimapDetector(BaseDetector)` that:
   - Wraps `le_lsd.LsdCC` (or configurable detector class)
   - Overrides `detect(camview)` — reads image from CameraView, passes to our LSD
   - Overrides `get_module_name()` → returns `"lsfm_lsd"`
   - Returns segments in LIMAP's `(N, 5)` format `[x1, y1, x2, y2, score]`
   - Inherits `detect_all_images()` from BaseDetector (loops over images) *(parallel with step 1-3)*

## Phase 3: Add ETH3D Data Loading to `lsfm.data`

5. In `python/lsfm/data.py`, add ETH3D loading support:
   - `TestImages.eth3d_scenes()` → list available scenes
   - `TestImages.eth3d_scene(name)` → returns dict with `images_dir`, `colmap_dir`, `cameras_json` paths
   - Validates scene directory structure exists *(parallel with step 4)*

## Phase 4: Create New `reconstruct_lines_multiview_full()` Function

6. In `python/lsfm/reconstruction.py`, add a new function that wraps LIMAP's full `line_triangulation` runner:
   - Uses `limap.pointsfm.read_infos_colmap()` to load ETH3D COLMAP data → gets `ImageCollection` + `neighbors` + `ranges` correctly (handles COLMAP pose convention properly, bypassing our `_build_limap_imagecols()` which has a different `t` convention)
   - Accepts optional `detector_override` parameter to inject `LsfmLimapDetector`
   - Calls `limap.runners.line_triangulation(cfg, imagecols, neighbors, ranges)`
   - Returns `LineTrack` objects with 3D lines, supporting view counts, track info
   - Exposes config overrides (`max_image_dim`, `n_neighbors`, detection method, etc.)
   - **Important**: ETH3D images are 6198×4129 — MUST downscale (suggest `max_image_dim=1600`) *(depends on steps 4, 5)*

## Phase 5: Create New Demo Notebook

7. Create `examples/notebooks/demo_multiview_reconstruction.ipynb` with sections:
   - **Cell 1**: Workspace setup + imports (same pattern as other notebooks)
   - **Cell 2**: Load ETH3D courtyard scene (38 images, known COLMAP poses)
   - **Cell 3**: Visualize scene — show sample images + camera positions
   - **Cell 4**: Run LIMAP's full pipeline with **built-in LSD** (pytlsd) as baseline
   - **Cell 5**: Run LIMAP's full pipeline with **our LSD** (LsfmLimapDetector)
   - **Cell 6**: Compare results — line counts, track lengths, num supporting views
   - **Cell 7**: 3D Rerun visualization — both reconstructions side by side
   - **Cell 8**: Statistics table *(depends on step 6)*

## Key Technical Details

### COLMAP Pose Convention (Critical!)

- ETH3D `cameras.json` stores `t` as COLMAP's **world-to-camera translation** (`t = -R @ center`)
- Our `_build_limap_imagecols()` assumes `t` is **camera center in world coords**
- **Solution**: Use LIMAP's own `read_infos_colmap()` for the new demo (no conversion needed)
- The existing stereo demo (MDB data) is unaffected (MDB's `t` IS the camera center)

### Custom Detector Integration

- LIMAP's `BaseDetector.detect(camview)` receives a `CameraView`, not an image
- `CameraView` provides `camview.read_image(True)` to get grayscale image
- Our subclass reads image from camview, passes to `le_lsd.LsdCC`, converts output
- LIMAP's `compute_2d_segs()` calls `detector.detect_all_images()` which we inherit

### ETH3D Image Size

- Courtyard images: 6198×4129 pixels (DSLR quality)
- MUST use `max_image_dim=1600` or similar to avoid OOM and keep runtime reasonable
- LIMAP handles resize internally via `imagecols.set_max_image_dim()`

### Injecting Custom Detector into LIMAP Pipeline

- LIMAP uses `limap.line2d.get_detector()` factory — hardcoded methods
- **Approach**: Monkey-patch `compute_2d_segs` or create thin wrapper that:
  1. Calls our detector's `detect_all_images()` to get `all_2d_segs`
  2. Also extracts descriptors with LIMAP's LBD extractor (reuse LIMAP's descriptor pipeline)
  3. Feeds both into `GlobalLineTriangulator`
- Alternative simpler approach: detect externally, save in LIMAP's segment folder format, set `load_det=True`

## Relevant Files

- `python/lsfm/limap_compat.py` — Add `LsfmLimapDetector(BaseDetector)` subclass
- `python/lsfm/reconstruction.py` — Add `reconstruct_lines_multiview_full()` wrapper
- `python/lsfm/data.py` — Add `eth3d_scenes()`, `eth3d_scene()` methods to `TestImages`
- `examples/notebooks/demo_3d_reconstruction.ipynb` → rename to `demo_stereo_reconstruction_limap.ipynb`
- `examples/notebooks/demo_multiview_reconstruction.ipynb` — NEW notebook
- `.venv/lib/python3.10/site-packages/limap/runners/line_triangulation.py` — Reference: full pipeline
- `.venv/lib/python3.10/site-packages/limap/line2d/base_detector.py` — Reference: BaseDetector interface
- `resources/datasets/ETH3D/courtyard/` — ETH3D data: 38 images + COLMAP calibration

## Verification

1. `ruff check python/lsfm/` — no lint errors
2. `bazel test //libs/...` — all existing tests pass
3. Run new notebook end-to-end: cells 1-8 execute without errors
4. LIMAP-LSD baseline produces >100 3D line tracks for courtyard (38 images)
5. Our LSD produces comparable or better line track counts
6. Rerun visualization shows sensible 3D reconstruction with camera frustums
7. No regressions in `demo_stereo_reconstruction_limap.ipynb` (renamed) or `demo_stereo_reconstruction.ipynb`

## Decisions

- Use LIMAP's full `line_triangulation` runner (not pairwise) — showcases real multi-view power
- Use `read_infos_colmap()` for ETH3D data loading — handles pose convention correctly
- Subclass `BaseDetector` for clean LIMAP integration (not pre-detect + load_det)
- Default scene: `courtyard` (38 images, strong architectural lines)
- Downscale to max_image_dim=1600 for practical runtime

## Further Considerations

1. **Descriptor extraction**: Our LSD produces gradients that can feed LBD descriptors, but LIMAP's descriptor pipeline expects its own interface. **Recommendation**: Use LIMAP's LBD extractor for both detectors — only swap the detector, keep descriptors consistent for fair comparison.
2. **Runtime**: Full LIMAP pipeline on 38 images (~1600px) with neighbors may take 2-5 minutes. Consider adding progress indicators or limiting to first N images for quick demo.
3. **The cameras.json convention issue**: Should we also fix cameras.json to store actual camera centers (matching MDB convention) to make it reusable with `reconstruct_lines_multiview()`? **Recommendation**: Yes, but as a separate task — document the convention clearly.
