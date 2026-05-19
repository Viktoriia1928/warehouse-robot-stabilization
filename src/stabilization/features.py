import cv2
import numpy as np


FEATURE_PARAMS = dict(
    maxCorners=500,
    qualityLevel=0.005,
    minDistance=12,
    blockSize=3,
)

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)

FB_ERR_THRESHOLD = 1.0


def detect_features(gray):
    return cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)


def track_features_fb(prev_gray, curr_gray, prev_pts):
    if prev_pts is None or len(prev_pts) == 0:
        return None, None

    curr_pts, status_f, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, curr_gray, prev_pts, None, **LK_PARAMS
    )
    if curr_pts is None or status_f is None:
        return None, None

    back_pts, status_b, _ = cv2.calcOpticalFlowPyrLK(
        curr_gray, prev_gray, curr_pts, None, **LK_PARAMS
    )
    if back_pts is None or status_b is None:
        return None, None

    good = (status_f.flatten() == 1) & (status_b.flatten() == 1)
    err = np.linalg.norm(back_pts.reshape(-1, 2) - prev_pts.reshape(-1, 2), axis=1)
    good &= err < FB_ERR_THRESHOLD

    if not np.any(good):
        return None, None
    return prev_pts[good], curr_pts[good]


def estimate_affine(prev_pts, curr_pts):
    if prev_pts is None or curr_pts is None or len(prev_pts) < 3:
        return None
    M, _ = cv2.estimateAffinePartial2D(
        prev_pts, curr_pts,
        method=cv2.RANSAC,
        ransacReprojThreshold=2.0,
        maxIters=3000,
        confidence=0.999,
        refineIters=15,
    )
    return M


def decompose_transform(M):
    if M is None:
        return 0.0, 0.0, 0.0
    return (
        float(M[0, 2]),
        float(M[1, 2]),
        float(np.arctan2(M[1, 0], M[0, 0])),
    )


def build_transform(dx, dy, da):
    return np.array(
        [
            [np.cos(da), -np.sin(da), dx],
            [np.sin(da),  np.cos(da), dy],
        ],
        dtype=np.float64,
    )
