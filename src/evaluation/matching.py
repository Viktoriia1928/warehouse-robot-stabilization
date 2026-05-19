from .iou import iou_xywh


def match_gt_to_predictions(gt_box, pred_list, iou_threshold=0.3):
    """Pick the prediction with the highest IoU above threshold."""
    best_pred = None
    best_iou = 0.0

    for pred in pred_list:
        score = iou_xywh(gt_box, pred["bbox"])
        if score > best_iou:
            best_iou = score
            best_pred = pred

    if best_iou >= iou_threshold:
        return True, best_pred, best_iou
    return False, None, best_iou
