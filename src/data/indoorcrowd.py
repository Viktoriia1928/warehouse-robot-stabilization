import os

import cv2
import numpy as np


def load_indoorcrowd(config="obj_det_seg", hf_token=None):
    """Load the IndoorCrowd dataset from Hugging Face."""
    from datasets import load_dataset
    if hf_token is not None:
        from huggingface_hub import login
        login(hf_token)
    return load_dataset("sebnae/IndoorCrowd", config)


def select_recording(ds, split_name, recording_id):
    """Return frames of a single recording, sorted by image_id."""
    data = ds[split_name]
    subset = [data[i] for i in range(len(data)) if data[i]["recording_id"] == recording_id]
    subset = sorted(subset, key=lambda x: x["image_id"])
    return subset


def build_video_from_split(subset, output_video, fps=10, frames_dir=None):
    """Render an mp4 from a list of dataset frames."""
    if frames_dir is None:
        frames_dir = os.path.splitext(output_video)[0] + "_frames"
    os.makedirs(frames_dir, exist_ok=True)

    for frame in subset:
        img = np.array(frame["image"])
        path = os.path.join(frames_dir, f"{frame['image_id']:05d}.jpg")
        cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    frame_files = sorted(
        os.path.join(frames_dir, f)
        for f in os.listdir(frames_dir)
        if f.endswith(".jpg")
    )

    first = cv2.imread(frame_files[0])
    h, w = first.shape[:2]

    writer = cv2.VideoWriter(
        output_video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
    )
    for path in frame_files:
        writer.write(cv2.imread(path))
    writer.release()

    return output_video, (w, h)


def extract_gt_tracks(subset, target_tids):
    """Extract {tid: {image_id: bbox_xywh}} for the requested targets."""
    gt_tracks = {tid: {} for tid in target_tids}
    for frame in subset:
        image_id = frame["image_id"]
        tids = frame["objects"]["track_id"]
        bboxes = frame["objects"]["bbox"]
        for box, tid in zip(bboxes, tids):
            if tid in gt_tracks:
                gt_tracks[tid][image_id] = box
    return gt_tracks
