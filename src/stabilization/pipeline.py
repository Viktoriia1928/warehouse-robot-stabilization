import os
import cv2
import numpy as np

from .features import (
    detect_features,
    track_features_fb,
    estimate_affine,
    decompose_transform,
    build_transform,
)
from .cleanup import clean_transforms
from .kalman import kalman_smooth_trajectory


def stabilize_v2(
    input_path,
    output_path,
    side_by_side_path=None,
    transforms_path=None,
    raw_transforms_path=None,
    trajectory_path=None,
    smoothed_trajectory_path=None,
    q=(1e-3, 1e-3, 1e-5),
    r=(1.0, 1.0, 1e-3),
    kalman_mode="rts",
    min_points=20,
    median_kernel=5,
    mad_k=4.0,
    verbose=True,
):
    """Shi-Tomasi + LK + RANSAC affine + Kalman RTS smoothing + warpAffine."""
    assert os.path.exists(input_path), input_path

    cap = cv2.VideoCapture(input_path)
    assert cap.isOpened(), f"cannot open {input_path}"
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS)) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if verbose:
        print(f"[V2] {w}x{h} @ {fps:.2f} fps, {n} frames; mode={kalman_mode}")
        print(f"[V2] q={q}  r={r}")

    ok, prev = cap.read()
    if not ok:
        raise RuntimeError(f"cannot read first frame of {input_path}")
    prev_gray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)

    transforms = []
    last_valid = np.zeros(3)
    counters = dict(detect_fail=0, track_fail=0, affine_fail=0, ok=0)

    for i in range(n - 1):
        prev_pts = detect_features(prev_gray)

        ok, curr = cap.read()
        if not ok:
            break
        curr_gray = cv2.cvtColor(curr, cv2.COLOR_BGR2GRAY)

        if prev_pts is None or len(prev_pts) < min_points:
            counters["detect_fail"] += 1
            transforms.append(last_valid.copy())
            prev_gray = curr_gray
            continue

        prev_good, curr_good = track_features_fb(prev_gray, curr_gray, prev_pts)
        if prev_good is None or len(prev_good) < min_points:
            counters["track_fail"] += 1
            transforms.append(last_valid.copy())
            prev_gray = curr_gray
            continue

        M = estimate_affine(prev_good, curr_good)
        if M is None:
            counters["affine_fail"] += 1
            transforms.append(last_valid.copy())
            prev_gray = curr_gray
            continue

        dx, dy, da = decompose_transform(M)
        last_valid = np.array([dx, dy, da])
        transforms.append(last_valid.copy())
        counters["ok"] += 1
        prev_gray = curr_gray

    transforms_raw = np.asarray(transforms, dtype=np.float64)
    if verbose:
        print(f"[V2 pass1] {counters}")

    transforms_clean = clean_transforms(
        transforms_raw, median_kernel=median_kernel, mad_k=mad_k, verbose=verbose
    )

    trajectory = np.cumsum(transforms_clean, axis=0)
    smoothed = kalman_smooth_trajectory(trajectory, q=q, r=r, mode=kalman_mode)
    transforms_smooth = transforms_clean + (smoothed - trajectory)

    cap.release()
    cap = cv2.VideoCapture(input_path)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".", exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
    sbs = None
    if side_by_side_path:
        sbs = cv2.VideoWriter(side_by_side_path, fourcc, fps, (2 * w, h))

    ok, frame = cap.read()
    writer.write(frame)
    if sbs is not None:
        sbs.write(np.hstack([frame, frame]))

    for i in range(n - 1):
        ok, frame = cap.read()
        if not ok or i >= len(transforms_smooth):
            break
        dx, dy, da = transforms_smooth[i]
        M = build_transform(dx, dy, da)
        stabilized = cv2.warpAffine(
            frame, M, (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        writer.write(stabilized)
        if sbs is not None:
            sbs.write(np.hstack([frame, stabilized]))

    cap.release()
    writer.release()
    if sbs is not None:
        sbs.release()

    if transforms_path:
        np.save(transforms_path, transforms_clean)
    if raw_transforms_path:
        np.save(raw_transforms_path, transforms_raw)
    if trajectory_path:
        np.save(trajectory_path, trajectory)
    if smoothed_trajectory_path:
        np.save(smoothed_trajectory_path, smoothed)

    if verbose:
        print(f"[V2 done] -> {output_path}")

    return dict(
        transforms=transforms_clean,
        transforms_raw=transforms_raw,
        trajectory=trajectory,
        smoothed_trajectory=smoothed,
        counters=counters,
        meta=dict(width=w, height=h, fps=fps, n_frames=n, q=q, r=r),
    )
