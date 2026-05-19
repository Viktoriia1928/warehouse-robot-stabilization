import json
import os

import cv2
from ultralytics import YOLO


def detect_and_track_botsort(
    input_video,
    output_video,
    imgsz=960,
    conf=0.20,
    iou=0.45,
    tracker_yaml="botsort.yaml",
    save_video=True,
    verbose=True,
):
    model = YOLO("yolo11n.pt")

    cap = cv2.VideoCapture(input_video)
    assert cap.isOpened(), f"cannot open video: {input_video}"

    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    writer = None
    if save_video:
        os.makedirs(os.path.dirname(os.path.abspath(output_video)) or ".", exist_ok=True)
        writer = cv2.VideoWriter(
            output_video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
        )

    tracks_log = []
    frame_idx = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model.track(
            source=frame,
            persist=True,
            tracker=tracker_yaml,
            imgsz=imgsz,
            conf=conf,
            iou=iou,
            verbose=False,
            classes=[0],
        )

        result = results[0]
        frame_tracks = []

        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy() if result.boxes.conf is not None else None
            ids = result.boxes.id.cpu().numpy() if result.boxes.id is not None else None
            classes = result.boxes.cls.cpu().numpy() if result.boxes.cls is not None else None

            for i in range(len(xyxy)):
                if classes is not None and int(classes[i]) != 0:
                    continue
                if ids is None:
                    continue

                x1, y1, x2, y2 = xyxy[i]
                tid = int(ids[i])
                tconf = float(confs[i]) if confs is not None else 1.0

                frame_tracks.append(
                    {
                        "id": tid,
                        "bbox": [float(x1), float(y1), float(x2), float(y2)],
                        "conf": tconf,
                    }
                )

                if writer is not None:
                    cv2.rectangle(
                        frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"id={tid} {tconf:.2f}",
                        (int(x1), max(20, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

        if writer is not None:
            writer.write(frame)

        tracks_log.append({"frame": frame_idx, "tracks": frame_tracks})
        frame_idx += 1
        if verbose and frame_idx % 100 == 0:
            print(f"[botsort] {frame_idx}/{n_frames}")

    cap.release()
    if writer is not None:
        writer.release()

    if save_video:
        out_json = output_video.replace(".mp4", "_tracks.json")
        with open(out_json, "w") as f:
            json.dump(
                {
                    "video": input_video,
                    "frames": tracks_log,
                    "meta": {"width": w, "height": h, "fps": fps, "n_frames": frame_idx},
                },
                f,
                indent=2,
            )

    return tracks_log
