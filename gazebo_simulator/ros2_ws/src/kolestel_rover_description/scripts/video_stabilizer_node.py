#!/usr/bin/env python3
"""Real-time video stabilizer for the rover's RGB camera.

Implements the 5 stabilization variants from the dissertation pipeline,
selectable through the `variant` ROS parameter:

    V1: LK features  + Moving-Average    smoothing
    V2: LK features  + 1D Kalman         smoothing      (production pick)
    V3: ORB features + Moving-Average    smoothing
    V4: robust LK    + Moving-Average    smoothing
    V4_pro / V5: robust LK + Kalman      smoothing
    none: passthrough (publishes input unchanged, for A/B comparison)

In a ROS node we cannot use the offline RTS smoother from the notebook,
so the Kalman variants here are causal (`filter` mode) — equivalent to
running the forward Kalman pass only.
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


# ----- helpers shared by variants -------------------------------------------

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
FB_ERR_THRESHOLD = 1.0


def _detect_gftt(gray, max_corners=500):
    return cv2.goodFeaturesToTrack(
        gray, mask=None, maxCorners=max_corners,
        qualityLevel=0.005, minDistance=12, blockSize=3,
    )


def _detect_orb(gray, n=600):
    orb = cv2.ORB_create(nfeatures=n, scaleFactor=1.2, nlevels=4)
    kps = orb.detect(gray, None)
    if not kps:
        return None
    pts = np.array([[k.pt] for k in kps], dtype=np.float32)
    return pts


def _track_lk(prev_gray, curr_gray, prev_pts):
    if prev_pts is None or len(prev_pts) == 0:
        return None, None
    curr_pts, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, None, **LK_PARAMS)
    if curr_pts is None or st is None:
        return None, None
    good = st.flatten() == 1
    if not np.any(good):
        return None, None
    return prev_pts[good], curr_pts[good]


def _track_lk_fb(prev_gray, curr_gray, prev_pts):
    """Forward-backward LK — discards drift-prone tracks."""
    if prev_pts is None or len(prev_pts) == 0:
        return None, None
    curr_pts, st_f, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, None, **LK_PARAMS)
    if curr_pts is None or st_f is None:
        return None, None
    prev_back, st_b, _ = cv2.calcOpticalFlowPyrLK(curr_gray, prev_gray, curr_pts, None, **LK_PARAMS)
    if prev_back is None or st_b is None:
        return None, None
    good = (st_f.flatten() == 1) & (st_b.flatten() == 1)
    err = np.linalg.norm(prev_back.reshape(-1, 2) - prev_pts.reshape(-1, 2), axis=1)
    good &= err < FB_ERR_THRESHOLD
    if not np.any(good):
        return None, None
    return prev_pts[good], curr_pts[good]


def _estimate_affine(prev_pts, curr_pts):
    if prev_pts is None or curr_pts is None or len(prev_pts) < 3:
        return None
    M, _ = cv2.estimateAffinePartial2D(
        prev_pts, curr_pts, method=cv2.RANSAC,
        ransacReprojThreshold=2.0, maxIters=3000,
        confidence=0.999, refineIters=15,
    )
    return M


def _decompose(M):
    if M is None:
        return 0.0, 0.0, 0.0
    return float(M[0, 2]), float(M[1, 2]), float(math.atan2(M[1, 0], M[0, 0]))


def _build(dx, dy, da):
    c, s = math.cos(da), math.sin(da)
    return np.array([[c, -s, dx], [s, c, dy]], dtype=np.float64)


# ----- causal 1D Kalman (constant-velocity) ---------------------------------

class _KalmanAxis:
    def __init__(self, q, r):
        self.F = np.array([[1.0, 1.0], [0.0, 1.0]])
        self.Q = q * np.array([[0.25, 0.5], [0.5, 1.0]])
        self.r = r
        self.x = None
        self.P = np.eye(2) * 10.0

    def update(self, z):
        if self.x is None:
            self.x = np.array([z, 0.0])
            return float(self.x[0])
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        S = P_pred[0, 0] + self.r
        K = P_pred[:, 0] / S
        self.x = x_pred + K * (z - x_pred[0])
        self.P = P_pred - np.outer(K, P_pred[0, :])
        return float(self.x[0])


# ----- causal moving-average ------------------------------------------------

class _MovingAvg:
    def __init__(self, window):
        self.window = max(int(window), 1)
        self.buf = []

    def update(self, z):
        self.buf.append(float(z))
        if len(self.buf) > self.window:
            self.buf.pop(0)
        return sum(self.buf) / len(self.buf)


# ----- main stabilizer state machine ---------------------------------------

class _Smoother:
    """Causal smoother over cumulative (dx, dy, da) trajectory."""

    def __init__(self, kind, ma_window=15, q=(1e-3, 1e-3, 1e-5), r=(1.0, 1.0, 1e-3)):
        self.kind = kind
        self.cum = np.zeros(3)
        if kind == 'ma':
            self.smoothers = [_MovingAvg(ma_window) for _ in range(3)]
        elif kind == 'kalman':
            self.smoothers = [_KalmanAxis(q[i], r[i]) for i in range(3)]
        else:
            raise ValueError(kind)

    def step(self, dx, dy, da):
        self.cum += np.array([dx, dy, da])
        smoothed = np.array([self.smoothers[i].update(self.cum[i]) for i in range(3)])
        # delta to apply on top of raw per-frame: smoothed_traj - raw_traj
        return smoothed - self.cum


VARIANTS = {
    # variant -> (feature_fn_name, tracker_name, smoother_kind)
    'V1':    ('gftt', 'lk',    'ma'),
    'V2':    ('gftt', 'lk',    'kalman'),
    'V3':    ('orb',  'lk_fb', 'ma'),
    'V4':    ('gftt', 'lk_fb', 'ma'),
    'V5':    ('gftt', 'lk_fb', 'kalman'),  # a.k.a. V4_pro
    'none':  (None, None, None),
}


class VideoStabilizerNode(Node):
    def __init__(self):
        super().__init__('video_stabilizer')

        self.declare_parameter('variant', 'V2')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('output_topic', '/camera/image_stabilized')
        self.declare_parameter('ma_window', 15)
        self.declare_parameter('q_xy', 1e-3)
        self.declare_parameter('q_a', 1e-5)
        self.declare_parameter('r_xy', 1.0)
        self.declare_parameter('r_a', 1e-3)
        self.declare_parameter('min_points', 20)

        variant = self.get_parameter('variant').get_parameter_value().string_value
        if variant not in VARIANTS:
            self.get_logger().warn(f"unknown variant '{variant}', falling back to V2")
            variant = 'V2'
        self.variant = variant
        feat_name, track_name, smooth_kind = VARIANTS[variant]

        self._detect = {
            None: None,
            'gftt': _detect_gftt,
            'orb': _detect_orb,
        }[feat_name]
        self._track = {
            None: None,
            'lk': _track_lk,
            'lk_fb': _track_lk_fb,
        }[track_name]

        if smooth_kind is None:
            self._smoother = None
        else:
            q = (
                self.get_parameter('q_xy').value,
                self.get_parameter('q_xy').value,
                self.get_parameter('q_a').value,
            )
            r = (
                self.get_parameter('r_xy').value,
                self.get_parameter('r_xy').value,
                self.get_parameter('r_a').value,
            )
            self._smoother = _Smoother(
                smooth_kind,
                ma_window=self.get_parameter('ma_window').value,
                q=q, r=r,
            )

        self.min_points = int(self.get_parameter('min_points').value)
        self.bridge = CvBridge()
        self.prev_gray = None
        self.frame_idx = 0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        in_topic = self.get_parameter('image_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.sub = self.create_subscription(Image, in_topic, self._on_image, qos)
        self.pub = self.create_publisher(Image, out_topic, qos)
        self.get_logger().info(
            f"stabilizer running (variant={self.variant}, in={in_topic}, out={out_topic})"
        )

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.variant == 'none' or self._smoother is None:
            out = frame
        else:
            out = self._stabilize_one(frame)

        out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)
        self.frame_idx += 1

    def _stabilize_one(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        prev_pts = self._detect(self.prev_gray) if self._detect else None
        dx = dy = da = 0.0
        if prev_pts is not None and len(prev_pts) >= self.min_points:
            pg, cg = self._track(self.prev_gray, gray, prev_pts) if self._track else (None, None)
            if pg is not None and len(pg) >= self.min_points:
                M = _estimate_affine(pg, cg)
                if M is not None:
                    dx, dy, da = _decompose(M)

        delta = self._smoother.step(dx, dy, da)
        # apply correction: transform = build(dx + ddx, dy + ddy, da + dda)
        # but in the notebook it's applied to the CURRENT frame: warp so the smoothed
        # frame replaces the noisy one. The transform we need warps `curr` so its
        # motion matches the smoothed trajectory.
        warp = _build(dx + delta[0], dy + delta[1], da + delta[2])
        stab = cv2.warpAffine(frame, warp, (w, h), borderMode=cv2.BORDER_REFLECT_101)

        self.prev_gray = gray
        return stab


def main():
    rclpy.init()
    node = VideoStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
