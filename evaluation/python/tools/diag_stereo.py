"""Diagnose stereo matching in reconstruct_lines_stereo."""

import le_lfd
import le_lsd
import numpy as np
from PIL import Image
from lsfm.data import TestImages


def main() -> None:
    """Run stereo matching diagnostics and print summary statistics."""
    ds = TestImages()
    img_l_path = ds.get("MDB/MiddEval3-H/Adirondack/im0.png")
    img_r_path = ds.get("MDB/MiddEval3-H/Adirondack/im1.png")

    # Load with PIL (as benchmark does)
    img_l = np.array(Image.open(img_l_path).convert("L"), dtype=np.uint8)
    img_r = np.array(Image.open(img_r_path).convert("L"), dtype=np.uint8)

    print(
        f"Image left:  {img_l.shape}, dtype={img_l.dtype}, range=[{img_l.min()}, {img_l.max()}]"
    )
    print(
        f"Image right: {img_r.shape}, dtype={img_r.dtype}, range=[{img_r.min()}, {img_r.max()}]"
    )

    det_l = le_lsd.LsdCC()
    det_l.detect(img_l)
    all_l = list(det_l.line_segments())
    segs_l = [s for s in all_l if s.length > 20.0]

    det_r = le_lsd.LsdCC()
    det_r.detect(img_r)
    all_r = list(det_r.line_segments())
    segs_r = [s for s in all_r if s.length > 20.0]

    print(
        f"\nSegments: {len(all_l)} raw L, {len(segs_l)} filtered L; {len(all_r)} raw R, {len(segs_r)} filtered R"
    )

    # Descriptors — use named accessors to avoid dependence on implicit list ordering
    gx_l = det_l.image_data_by_name("gx").astype(np.float32)
    gy_l = det_l.image_data_by_name("gy").astype(np.float32)
    gx_r = det_r.image_data_by_name("gx").astype(np.float32)
    gy_r = det_r.image_data_by_name("gy").astype(np.float32)

    lbd_l = le_lfd.FdcLBD(gx_l, gy_l)
    desc_l = lbd_l.create_list(segs_l)
    lbd_r = le_lfd.FdcLBD(gx_r, gy_r)
    desc_r = lbd_r.create_list(segs_r)

    print(f"Descriptors: {len(desc_l)} L, {len(desc_r)} R")

    # Raw matching (no filter)
    matcher_raw = le_lfd.BruteForceLBD()
    matcher_raw.train(desc_l, desc_r)
    best_raw = matcher_raw.best()
    non_nan_raw = sum(1 for m in best_raw if not np.isnan(m.distance))
    print(f"\nRaw matching (no filter): {non_nan_raw} non-NaN out of {len(best_raw)}")

    # Stereo filtered matching
    sf = le_lfd.StereoLineFilter(
        height=img_l.shape[0], max_dis_px=4.0, angle_th=5.0, min_y_overlap=0.3
    )
    sf.train(segs_l, segs_r)

    matcher_sf = le_lfd.BruteForceLBD()
    matcher_sf.train_filtered_stereo(desc_l, desc_r, sf)
    best_sf = matcher_sf.best()
    non_nan_sf = sum(1 for m in best_sf if not np.isnan(m.distance))
    print(f"Stereo-filtered matching: {non_nan_sf} non-NaN out of {len(best_sf)}")

    # Rotation filter
    rf = le_lfd.GlobalRotationFilter()
    rf.train(segs_l, segs_r)

    pairs = []
    for i, m in enumerate(best_sf):
        if np.isnan(m.distance):
            continue
        if not rf.filter(i, m.match_idx):
            continue
        pairs.append((i, m.match_idx))
    print(f"After rotation filter: {len(pairs)} pairs")


if __name__ == "__main__":
    main()
