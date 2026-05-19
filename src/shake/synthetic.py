import math
import random

import cv2
import numpy as np


def add_realistic_shake(
    input_video,
    output_video,
    sin_amp_trans=10.0,
    sin_amp_rot=0.015,
    freqs=(2.5, 4.0, 7.0),
    white_std=1.5,
    burst_prob=0.015,
    burst_mag=12.0,
    seed=42,
    fps_override=None,
    border_mode=cv2.BORDER_REPLICATE,
):
    """Inject multi-frequency vibration + occasional bursts via warpAffine."""
    random.seed(seed)
    np.random.seed(seed)

    cap = cv2.VideoCapture(input_video)
    assert cap.isOpened(), f"cannot open {input_video}"

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps_override is not None:
        fps = fps_override
    if fps <= 0:
        fps = 10.0

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = cv2.VideoWriter(
        output_video,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (w, h),
    )

    t = 0
    burst_dx = burst_dy = burst_da = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        time_s = t / fps
        dx = dy = da = 0.0

        for i, f in enumerate(freqs):
            phase1 = 0.7 * i
            phase2 = 1.1 * i
            phase3 = 0.9 * i
            dx += (sin_amp_trans / (i + 1)) * math.sin(2 * math.pi * f * time_s + phase1)
            dy += (0.8 * sin_amp_trans / (i + 1)) * math.sin(2 * math.pi * (f * 0.9) * time_s + phase2)
            da += (sin_amp_rot / (i + 1)) * math.sin(2 * math.pi * (f * 1.1) * time_s + phase3)

        dx += np.random.normal(0, white_std)
        dy += np.random.normal(0, white_std)
        da += np.random.normal(0, sin_amp_rot * 0.15)

        if random.random() < burst_prob:
            burst_dx += np.random.uniform(-burst_mag, burst_mag)
            burst_dy += np.random.uniform(-burst_mag, burst_mag)
            burst_da += np.random.uniform(-sin_amp_rot * 3, sin_amp_rot * 3)

        dx += burst_dx
        dy += burst_dy
        da += burst_da

        burst_dx *= 0.85
        burst_dy *= 0.85
        burst_da *= 0.85

        cx, cy = w / 2.0, h / 2.0
        cos_a = math.cos(da)
        sin_a = math.sin(da)

        M = np.array(
            [
                [cos_a, -sin_a, dx + cx - cos_a * cx + sin_a * cy],
                [sin_a,  cos_a, dy + cy - sin_a * cx - cos_a * cy],
            ],
            dtype=np.float32,
        )

        shaken = cv2.warpAffine(
            frame, M, (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=border_mode,
        )
        writer.write(shaken)
        t += 1

    cap.release()
    writer.release()
