from collections import Counter

from .matching import match_gt_to_predictions


def compute_longest_true_run(bool_list):
    best = 0
    cur = 0
    for v in bool_list:
        if v:
            cur += 1
            best = max(best, cur)
        else:
            cur = 0
    return best


def compute_lost_segments(found_list):
    lost = 0
    prev = None
    for v in found_list:
        if prev is True and v is False:
            lost += 1
        prev = v
    return lost


def compute_id_switches(pred_id_seq):
    switches = 0
    prev_id = None
    for pid in pred_id_seq:
        if pid is None:
            continue
        if prev_id is None:
            prev_id = pid
            continue
        if pid != prev_id:
            switches += 1
        prev_id = pid
    return switches


def evaluate_gt_target(gt_track_dict, pred_by_image_id, iou_threshold=0.3):
    """GT-based tracking metrics for a single physical target."""
    image_ids = sorted(gt_track_dict.keys())

    found_list = []
    matched_ious = []
    pred_id_seq = []
    per_frame = []

    for image_id in image_ids:
        gt_box = gt_track_dict[image_id]
        pred_list = pred_by_image_id.get(image_id, [])
        matched, best_pred, best_iou = match_gt_to_predictions(
            gt_box, pred_list, iou_threshold=iou_threshold
        )

        if matched:
            found_list.append(True)
            matched_ious.append(best_iou)
            pred_id_seq.append(best_pred["track_id"])
            per_frame.append({
                "image_id": image_id,
                "gt_bbox": gt_box,
                "found": True,
                "iou": best_iou,
                "pred_track_id": best_pred["track_id"],
                "pred_bbox": best_pred["bbox"],
                "pred_score": best_pred.get("score"),
            })
        else:
            found_list.append(False)
            pred_id_seq.append(None)
            per_frame.append({
                "image_id": image_id,
                "gt_bbox": gt_box,
                "found": False,
                "iou": best_iou,
                "pred_track_id": None,
                "pred_bbox": None,
                "pred_score": None,
            })

    total = len(image_ids)
    matched = sum(found_list)
    lock_rate = matched / total if total else 0.0
    mean_iou = sum(matched_ious) / len(matched_ious) if matched_ious else 0.0

    longest_run = compute_longest_true_run(found_list)
    lost_segments = compute_lost_segments(found_list)
    id_switches = compute_id_switches(pred_id_seq)

    assigned_ids = [pid for pid in pred_id_seq if pid is not None]
    unique_ids = len(set(assigned_ids))

    if assigned_ids:
        counter = Counter(assigned_ids)
        dominant_id, dominant_count = counter.most_common(1)[0]
        dominant_purity = dominant_count / len(assigned_ids)
    else:
        dominant_id = None
        dominant_purity = 0.0

    return {
        "total_gt_frames": total,
        "matched_frames": matched,
        "lock_rate": lock_rate,
        "mean_iou": mean_iou,
        "lost_segments": lost_segments,
        "longest_continuous_run": longest_run,
        "id_switches": id_switches,
        "unique_assigned_pred_ids": unique_ids,
        "dominant_pred_id": dominant_id,
        "dominant_id_purity": dominant_purity,
        "per_frame": per_frame,
    }


def tracks_log_to_pred_by_image_id(tracks_log, frame_id_offset=1):
    """Convert tracker output into the {image_id: [predictions]} format expected by evaluate_gt_target."""
    pred_by_image_id = {}
    for entry in tracks_log:
        frame_idx = entry["frame"]
        image_id = frame_idx + frame_id_offset
        preds = []
        for tr in entry["tracks"]:
            x1, y1, x2, y2 = tr["bbox"]
            preds.append({
                "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                "track_id": int(tr["id"]),
                "score": float(tr["conf"]),
            })
        pred_by_image_id[image_id] = preds
    return pred_by_image_id
