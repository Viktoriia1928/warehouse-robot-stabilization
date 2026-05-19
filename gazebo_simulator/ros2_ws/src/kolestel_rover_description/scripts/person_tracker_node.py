#!/usr/bin/env python3
"""YOLO11n + OC-SORT person tracker for the rover's camera stream.

Subscribes:
    /camera/image_stabilized   sensor_msgs/Image   (input)

Publishes:
    /tracking/image            sensor_msgs/Image          (annotated)
    /tracking/persons          vision_msgs/Detection2DArray (id stored in Detection2D.id)
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

PERSON_CLASS = 0


def _import_yolo():
    from ultralytics import YOLO  # noqa: WPS433
    return YOLO


def _import_ocsort():
    # boxmot >=18 no longer re-exports OcSort at top level
    try:
        from boxmot.trackers.ocsort.ocsort import OcSort  # type: ignore
        return OcSort
    except ImportError:
        pass
    try:
        from boxmot import OcSort  # type: ignore
        return OcSort
    except ImportError:
        from boxmot import OCSORT  # type: ignore
        return OCSORT


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker')

        self.declare_parameter('image_topic', '/camera/image_stabilized')
        self.declare_parameter('annotated_topic', '/tracking/image')
        self.declare_parameter('detections_topic', '/tracking/persons')
        self.declare_parameter('yolo_weights', 'yolo11n.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('asso_threshold', 0.3)

        weights = self.get_parameter('yolo_weights').value
        YOLO = _import_yolo()
        OcSort = _import_ocsort()

        self.detector = YOLO(weights)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.imgsz = int(self.get_parameter('imgsz').value)

        self.tracker = OcSort(
            det_thresh=self.conf,
            max_age=int(self.get_parameter('max_age').value),
            min_hits=int(self.get_parameter('min_hits').value),
            iou_threshold=float(self.get_parameter('asso_threshold').value),
            delta_t=3, inertia=0.2, use_byte=False,
        )

        self.bridge = CvBridge()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, qos
        )
        self.pub_img = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, qos
        )
        self.pub_det = self.create_publisher(
            Detection2DArray, self.get_parameter('detections_topic').value, 10
        )
        self.get_logger().info('person_tracker ready (YOLO + OC-SORT)')

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        r = self.detector(
            frame, classes=[PERSON_CLASS], conf=self.conf, iou=self.iou,
            imgsz=self.imgsz, verbose=False,
        )[0]
        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.cpu().numpy()
            cf = r.boxes.conf.cpu().numpy()
            cl = r.boxes.cls.cpu().numpy()
            dets = np.column_stack([xyxy, cf, cl]).astype(np.float32)
        else:
            dets = np.empty((0, 6), dtype=np.float32)

        tracks = self.tracker.update(dets, frame)

        annotated = frame.copy()
        det_array = Detection2DArray()
        det_array.header = msg.header

        for t in tracks:
            x1, y1, x2, y2 = map(float, t[:4])
            tid = int(t[4])
            tconf = float(t[5]) if len(t) > 5 else 1.0
            color = ((tid * 41) % 255, (tid * 73) % 255, (tid * 113) % 255)
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(
                annotated, f"ID {tid} {tconf:.2f}", (int(x1), max(15, int(y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
            )

            d = Detection2D()
            d.header = msg.header
            d.id = str(tid)
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = tconf
            d.results.append(hyp)
            det_array.detections.append(d)

        cv2.putText(
            annotated, f"persons: {len(tracks)}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_img.publish(out_msg)
        self.pub_det.publish(det_array)


def main():
    rclpy.init()
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
