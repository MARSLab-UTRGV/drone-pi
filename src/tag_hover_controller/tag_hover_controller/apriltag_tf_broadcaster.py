#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def to_int_id(det):
    # Accept both scalar and list-like id fields
    for attr in ('id', 'ids', 'fiducial_id', 'fiducial_ids'):
        if hasattr(det, attr):
            val = getattr(det, attr)
            try:
                # list/tuple/array case
                return int(val[0]) if len(val) > 0 else 0
            except TypeError:
                # scalar case
                try:
                    return int(val)
                except Exception:
                    pass
    return 0

def extract_pose_and_header(det):
    """
    Return (pose, header) from a detection, tolerating different layouts:
      - det.pose.pose.pose (PoseWithCovarianceStamped)
      - det.pose.pose      (PoseStamped or PoseWithCovarianceStamped.pose)
      - det.pose           (Pose)
    """
    # Case A: PoseWithCovarianceStamped (most common: det.pose.pose.pose)
    try:
        return det.pose.pose.pose, det.pose.header
    except AttributeError:
        pass
    # Case B: PoseStamped (det.pose.pose with header in det.pose.header)
    try:
        if hasattr(det.pose, 'pose') and hasattr(det.pose, 'header'):
            return det.pose.pose, det.pose.header
    except AttributeError:
        pass
    # Case C: Plain Pose (no header)
    try:
        if hasattr(det.pose, 'position') and hasattr(det.pose, 'orientation'):
            return det.pose, None
    except AttributeError:
        pass

    # If we still can't parse, print what we saw for quick diagnosis
    raise RuntimeError(
        f"Unsupported pose layout. det.pose attrs={dir(det.pose) if hasattr(det, 'pose') else 'NO pose'}"
    )

class TagTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tag_tf_broadcaster')
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').get_parameter_value().string_value
        self.tag_prefix   = self.declare_parameter('tag_prefix', 'tag36h11').get_parameter_value().string_value
        self.detections_topic = self.declare_parameter('detections_topic', '/detections').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(AprilTagDetectionArray, self.detections_topic, self.cb, qos)

        self.get_logger().info(f"Started. Listening on {self.detections_topic}")
        self.get_logger().info(f"camera_frame={self.camera_frame}, tag_prefix={self.tag_prefix}")

    def cb(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return
        for det in msg.detections:
            try:
                tag_id = to_int_id(det)
                pose, header = extract_pose_and_header(det)

                t = TransformStamped()
                if header is not None and hasattr(header, 'stamp'):
                    t.header.stamp = header.stamp
                    t.header.frame_id = getattr(header, 'frame_id', '') or self.camera_frame
                else:
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.camera_frame

                t.child_frame_id = f"{self.tag_prefix}:{tag_id}"
                t.transform.translation.x = float(pose.position.x)
                t.transform.translation.y = float(pose.position.y)
                t.transform.translation.z = float(pose.position.z)
                t.transform.rotation = pose.orientation

                self.br.sendTransform(t)
                self.get_logger().info(
                    f"TF {t.header.frame_id} -> {t.child_frame_id} "
                    f"xyz=({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, {t.transform.translation.z:.3f})"
                )
            except Exception as e:
                self.get_logger().warn(f"Skipping detection: {e}")

def main():
    rclpy.init()
    node = TagTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

