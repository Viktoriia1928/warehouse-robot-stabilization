#!/usr/bin/env python3
"""Visual person-follower.

Locks onto the longest-lived track ID seen so far (matches the notebook's
target-locking logic) and drives the rover toward it: angular velocity
is proportional to the target's horizontal offset from the image center;
linear velocity is gated by the bounding box height as a crude distance
proxy (smaller bbox = further away = faster).

Subscribes:
    /tracking/persons     vision_msgs/Detection2DArray
    /camera/camera_info   sensor_msgs/CameraInfo   (for image width)

Publishes:
    /cmd_vel              geometry_msgs/Twist
    /tracking/target_id   std_msgs/Int32
"""
from collections import Counter

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32
from vision_msgs.msg import Detection2DArray


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('detections_topic', '/tracking/persons')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.55)
        self.declare_parameter('far_boost', 1.8)
        self.declare_parameter('angular_gain', 0.0035)
        self.declare_parameter('max_angular_speed', 2.5)
        self.declare_parameter('turn_priority_offset_frac', 0.25)
        self.declare_parameter('stop_bbox_height_frac', 0.65)
        self.declare_parameter('min_bbox_height_frac', 0.12)
        self.declare_parameter('lost_grace_frames', 30)
        self.declare_parameter('relock_after_lost', True)

        self.image_width = 1280
        self.image_height = 720
        self.target_id: int | None = None
        self.id_seen_count: Counter[int] = Counter()
        self.lost_counter = 0

        self.sub_det = self.create_subscription(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            self._on_detections, 10,
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_info, 5,
        )

        self.pub_cmd = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )
        self.pub_target = self.create_publisher(
            Int32, '/tracking/target_id', 10
        )

        self.get_logger().info('person_follower ready')

    def _on_info(self, msg: CameraInfo):
        if msg.width > 0:
            self.image_width = msg.width
        if msg.height > 0:
            self.image_height = msg.height

    def _pick_target(self, current_ids):
        """Longest-lived id heuristic. Locks the first id that hits a
        few-frame threshold. After the locked id disappears for >grace,
        we re-lock to whichever current id has been seen the longest."""
        for tid in current_ids:
            self.id_seen_count[tid] += 1

        if self.target_id is not None and self.target_id in current_ids:
            self.lost_counter = 0
            return self.target_id

        if self.target_id is None:
            if current_ids:
                best = max(current_ids, key=lambda i: self.id_seen_count[i])
                if self.id_seen_count[best] >= 3:
                    self.target_id = best
                    self.get_logger().info(f"locked target id={self.target_id}")
            return self.target_id

        # had a target, but it's not in the current frame
        self.lost_counter += 1
        grace = int(self.get_parameter('lost_grace_frames').value)
        if self.lost_counter > grace and bool(self.get_parameter('relock_after_lost').value):
            old = self.target_id
            self.target_id = None
            self.lost_counter = 0
            self.get_logger().warn(f"target id={old} lost; will re-lock")
        return self.target_id

    def _on_detections(self, msg: Detection2DArray):
        cmd = Twist()

        # collect (id, bbox) for this frame
        per_id = {}
        for det in msg.detections:
            try:
                tid = int(det.id)
            except (TypeError, ValueError):
                continue
            per_id[tid] = det.bbox

        target = self._pick_target(set(per_id.keys()))

        if target is None or target not in per_id:
            self.pub_cmd.publish(cmd)  # zero -> stop
            self.pub_target.publish(Int32(data=-1))
            return

        bbox = per_id[target]
        cx = bbox.center.position.x
        h = bbox.size_y
        img_w = float(self.image_width) or 1.0
        img_h = float(self.image_height) or 1.0

        # angular: drive offset to zero. Saturate at max_angular_speed.
        offset = cx - img_w / 2.0
        offset_frac = abs(offset) / max(img_w / 2.0, 1.0)
        max_omega = float(self.get_parameter('max_angular_speed').value)
        raw_omega = -float(self.get_parameter('angular_gain').value) * offset
        cmd.angular.z = max(-max_omega, min(max_omega, raw_omega))

        # linear: drive forward unless the person already fills the frame.
        # When the target is far (small bbox) we boost above `linear_speed`
        # to close the gap; when close, we ramp linearly down to a stop.
        # When the target is far off-center, we cut linear speed so the
        # robot turns in place instead of driving past the person.
        h_frac = h / img_h
        stop_frac = float(self.get_parameter('stop_bbox_height_frac').value)
        min_frac = float(self.get_parameter('min_bbox_height_frac').value)
        max_v = float(self.get_parameter('linear_speed').value)
        far_boost = float(self.get_parameter('far_boost').value)
        turn_prio = float(self.get_parameter('turn_priority_offset_frac').value)

        if h_frac >= stop_frac:
            base_v = 0.0
        elif h_frac >= min_frac:
            t = (stop_frac - h_frac) / (stop_frac - min_frac)
            base_v = max_v * max(0.0, min(1.0, t))
        else:
            far_factor = 1.0 + far_boost * (min_frac - h_frac) / max(min_frac, 1e-6)
            base_v = max_v * far_factor

        if offset_frac > turn_prio:
            # off-axis: scale linear down so robot can spin onto target
            scale = max(0.0, 1.0 - (offset_frac - turn_prio) / (1.0 - turn_prio))
            base_v *= scale
        cmd.linear.x = base_v

        self.pub_cmd.publish(cmd)
        self.pub_target.publish(Int32(data=int(target)))


def main():
    rclpy.init()
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
