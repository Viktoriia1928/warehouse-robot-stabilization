def bbox_xywh_to_xyxy(box):
    x, y, w, h = box
    return [x, y, x + w, y + h]


def iou_xywh(box_a, box_b):
    ax1, ay1, ax2, ay2 = bbox_xywh_to_xyxy(box_a)
    bx1, by1, bx2, by2 = bbox_xywh_to_xyxy(box_b)

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter

    return inter / union if union > 0 else 0.0
