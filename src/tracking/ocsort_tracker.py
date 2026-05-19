import json
import os
import time

import cv2
import numpy as np
from ultralytics import YOLO

try:
    from boxmot import OcSort as _OCSort
except ImportError:
    try:
        from boxmot.trackers.ocsort.ocsort import OcSort as _OCSort
    except ImportError:
        from boxmot import OCSORT as _OCSort

PERSON_CLASS = 0


def detect_and_track_ocsort(
    input_video,
    output_video,
    yolo_weights="yolo11n.pt",
    conf=0.25,
    iou=0.45,
    imgsz=640,
    save_video=True,
    verbose=True,
):
    detector = YOLO(yolo_weights)
    tracker = _OCSort(
        det_thresh=conf,
        max_age=30,
        min_hits=3,
        asso_threshold=0.3,
        delta_t=3,
        inertia=0.2,
        use_byte=False,
    )

    cap = cv2.VideoCapture(input_video)
    assert cap.isOpened(), f"cannot open {input_video}"
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if verbose:
        print(f"[ocsort] {os.path.basename(input_video)}: {w}x{h} @ {fps:.1f} fps")

    writer = None
    if save_video:
        os.makedirs(os.path.dirname(os.path.abspath(output_video)) or ".", exist_ok=True)
        writer = cv2.VideoWriter(
            output_video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
        )

    tracks_log = []
    frame_times = []
    frame_idx = 0
    t_total = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        t0 = time.time()
        result = detector(
            frame,
            classes=[PERSON_CLASS],
            conf=conf,
            iou=iou,
            imgsz=imgsz,
            verbose=False,
        )[0]

        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            cf = result.boxes.conf.cpu().numpy()
            cl = result.boxes.cls.cpu().numpy()
            detections = np.column_stack([xyxy, cf, cl]).astype(np.float32)
        else:
            detections = np.empty((0, 6), dtype=np.float32)

        tracks = tracker.update(detections, frame)
        frame_times.append(time.time() - t0)

        frame_tracks = []
        for tr in tracks:
            x1, y1, x2, y2 = map(int, tr[:4])
            tid = int(tr[4])
            tconf = float(tr[5]) if len(tr) > 5 else 1.0
            frame_tracks.append(
                {"id": tid, "bbox": [x1, y1, x2, y2], "conf": tconf}
            )

            if writer is not None:
                color = ((tid * 41) % 255, (tid * 73) % 255, (tid * 113) % 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame, f"ID {tid} {tconf:.2f}",
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
                )

        if writer is not None:
            writer.write(frame)

        tracks_log.append({"frame": frame_idx, "tracks": frame_tracks})
        frame_idx += 1

    cap.release()
    if writer is not None:
        writer.release()

    summary = _summarize(tracks_log, frame_times, w, h, fps, frame_idx,
                         os.path.basename(input_video), time.time() - t_total)

    if save_video:
        with open(output_video.replace(".mp4", "_tracks.json"), "w") as f:
            json.dump(
                {
                    "video": input_video,
                    "frames": tracks_log,
                    "meta": {"width": w, "height": h, "fps": fps, "n_frames": frame_idx},
                },
                f,
                indent=2,
            )
        with open(output_video.replace(".mp4", "_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

    if verbose:
        print(f"[ocsort done] {summary}")

    return summary, tracks_log


def _summarize(tracks_log, frame_times, w, h, fps, n_frames, name, total_runtime):
    life = {}
    for entry in tracks_log:
        for tr in entry["tracks"]:
            life[tr["id"]] = life.get(tr["id"], 0) + 1

    confs_all = [tr["conf"] for e in tracks_log for tr in e["tracks"]]
    counts = [len(e["tracks"]) for e in tracks_log]

    return {
        "video": name,
        "total_frames": n_frames,
        "unique_track_ids": len(life),
        "mean_track_lifespan_frames": round(
            float(np.mean(list(life.values()))) if life else 0.0, 2
        ),
        "max_track_lifespan_frames": int(max(life.values())) if life else 0,
        "frames_with_tracks": int(sum(1 for c in counts if c > 0)),
        "avg_tracks_per_frame": round(float(np.mean(counts)) if counts else 0.0, 3),
        "mean_confidence": round(float(np.mean(confs_all)) if confs_all else 0.0, 3),
        "pipeline_ms_per_frame": round(float(np.mean(frame_times) * 1000) if frame_times else 0.0, 2),
        "total_runtime_s": round(total_runtime, 2),
    }
