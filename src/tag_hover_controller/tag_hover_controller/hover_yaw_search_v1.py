#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class HoverYawSearch(Node):
    """
    3-DOF Semi-autonomous controller with yaw + distance locking:

    - SUB: /mavros/state (mavros_msgs/State)
    - SUB: /detections (apriltag_ros/AprilTagDetectionArray)  [for tag size]
    - TF:  camera -> tag36h11:0   (from apriltag_pnp_broadcaster)
    - PUB: /hover_yaw_cmd (geometry_msgs/Twist)               [debug]
    - PUB: /mavros/setpoint_velocity/cmd_vel_unstamped (Twist) [to FCU]

    Modes:
      SEARCH: constant yaw rate (search_yaw) until tag seen
      LOCK:   yaw to keep tag centered + distance control to maintain target tag size
    """

    def __init__(self):
        super().__init__('hover_yaw_search')

        # Parameters
        self.declare_parameter('mode', 'SEARCH')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('search_yaw', 0.25)          # rad/s
        self.declare_parameter('lock_k_yaw', 0.1)           # P gain for yaw
        self.declare_parameter('lock_k_distance', 0.2)      # P gain for forward/backward (m/s per meter error)
        self.declare_parameter('lock_k_lateral', 0.1)       # P gain for left/right (m/s per meter error)
        self.declare_parameter('lock_k_vertical', 0.1)      # P gain for up/down (m/s per meter error)
        self.declare_parameter('yaw_align_threshold', 0.1)  # Radians; only move forward/lateral when |yaw_error| < this
        self.declare_parameter('target_distance', 2.0)      # Target distance from tag in meters
        self.declare_parameter('max_forward_vel', 0.5)      # m/s clamp for forward/backward
        self.declare_parameter('max_lateral_vel', 0.5)      # m/s clamp for left/right
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('max_yaw_rate', 0.6)         # rad/s clamp
        self.declare_parameter('mavros_wait_timeout', 10.0)  # seconds to wait for MAVROS
        self.declare_parameter('mavros_prefix', '/mavros')   # base path for MAVROS topics

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.search_yaw = self.get_parameter('search_yaw').get_parameter_value().double_value
        self.lock_k_yaw = self.get_parameter('lock_k_yaw').get_parameter_value().double_value
        self.lock_k_distance = self.get_parameter('lock_k_distance').get_parameter_value().double_value
        self.lock_k_lateral = self.get_parameter('lock_k_lateral').get_parameter_value().double_value
        self.lock_k_vertical = self.get_parameter('lock_k_vertical').get_parameter_value().double_value
        self.yaw_align_threshold = self.get_parameter('yaw_align_threshold').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.max_forward_vel = self.get_parameter('max_forward_vel').get_parameter_value().double_value
        self.max_lateral_vel = self.get_parameter('max_lateral_vel').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.mavros_wait_timeout = self.get_parameter('mavros_wait_timeout').get_parameter_value().double_value
        # Normalize MAVROS topic prefix (leading slash, no trailing slash)
        raw_prefix = self.get_parameter('mavros_prefix').get_parameter_value().string_value
        raw_prefix = raw_prefix if raw_prefix is not None else '/mavros'
        if not raw_prefix.startswith('/'):
            raw_prefix = '/' + raw_prefix
        self.mavros_prefix = raw_prefix.rstrip('/') or ''

        # State from MAVROS
        self._state: State | None = None
        self._have_state = False
        self._startup_time = rclpy.clock.Clock().now()
        self._mavros_ready_logged = False

        state_topic = f"{self.mavros_prefix}/state" if self.mavros_prefix else '/mavros/state'
        self._state_sub = self.create_subscription(
            State,
            state_topic,
            self._state_cb,
            10
        )

        # AprilTag detections for tag size measurement
        # Use generic message subscription to avoid import errors
        self._detections = None
        try:
            # Try to get the message type from apriltag_msgs (not apriltag_ros)
            msg_type = get_message('apriltag_msgs/msg/AprilTagDetectionArray')
            self._detections_sub = self.create_subscription(
                msg_type,
                '/detections',
                self._detections_cb,
                10
            )
        except Exception as e:
            self.get_logger().warn(f"Could not subscribe to apriltag detections: {e}. Distance control disabled.")

        # Debug command topic
        self._hover_cmd_pub = self.create_publisher(
            Twist,
            '/hover_yaw_cmd',
            10
        )

        # Velocity setpoint to FCU
        vel_topic = f"{self.mavros_prefix}/setpoint_velocity/cmd_vel_unstamped" if self.mavros_prefix else '/mavros/setpoint_velocity/cmd_vel_unstamped'
        self._vel_pub = self.create_publisher(
            Twist,
            vel_topic,
            10
        )

        # TF listener for camera -> tag
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Timer
        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"hover_yaw_search started. mode={self.mode}, "
            f"rate_hz={self.rate_hz}, search_yaw={self.search_yaw} rad/s, "
            f"lock_k_yaw={self.lock_k_yaw}, lock_k_distance={self.lock_k_distance} (m/s per m), "
            f"lock_k_lateral={self.lock_k_lateral} (m/s per m), yaw_align_threshold={self.yaw_align_threshold} rad, "
            f"target_distance={self.target_distance} m, "
            f"camera_frame={self.camera_frame}, tag_frame={self.tag_frame}"
        )

    # ------------------------- Callbacks -------------------------

    def _state_cb(self, msg: State):
        self._state = msg
        self._have_state = True

    def _detections_cb(self, msg):
        """Callback for AprilTag detections (generic message)."""
        self._detections = msg

    # ------------------------- Helper Methods -------------------------

    def _get_tag_size(self) -> float | None:
        """
        DEPRECATED: Use 3D distance from TF instead.
        Kept for reference only.
        """
        return None

    # ------------------------- Core loop -------------------------

    def _on_timer(self):
        # 1) Wait for MAVROS to initialize
        if not self._have_state or self._state is None:
            elapsed = (rclpy.clock.Clock().now() - self._startup_time).nanoseconds / 1e9
            if not self._mavros_ready_logged and elapsed < self.mavros_wait_timeout:
                self.get_logger().info(f'Waiting for MAVROS initialization... ({elapsed:.1f}s/{self.mavros_wait_timeout:.1f}s)')
                return
            elif elapsed >= self.mavros_wait_timeout:
                self.get_logger().error(f'MAVROS did not initialize within {self.mavros_wait_timeout}s timeout')
                return
            else:
                self._mavros_ready_logged = True
                self.get_logger().info(f'MAVROS state received after {elapsed:.1f}s')
        
        if not self._state.connected:
            self.get_logger().warn('MAVROS connected: False', throttle_duration_sec=2.0)
            return
        
        if not self._mavros_ready_logged:
            self._mavros_ready_logged = True
            self.get_logger().info('MAVROS connected and ready')

        # For now, we only *test* in GUIDED/LOITER/GUIDED_NOGPS
        mode = self._state.mode.upper() if self._state.mode else ''
        if mode not in ['GUIDED', 'GUIDED_NOGPS', 'LOITER']:
            self.get_logger().warn(
                f'FCU mode is {mode}, expected GUIDED/GUIDED_NOGPS/LOITER for testing.',
                throttle_duration_sec=5.0
            )
            return

        cmd = Twist()

        if self.mode.upper() == 'LOCK':
            # Try to get camera -> tag transform
            try:
                # Latest transform
                tf = self._tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.tag_frame,
                    Time()
                )

                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z

                # Basic sanity
                if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z) or abs(z) < 1e-3:
                    raise TransformException("Non-finite TF values or z ≈ 0")

                # === 4-DOF RELATIVE POSE REGULATION ===
                # Target relative pose (in camera frame):
                # x = 0 (centered horizontally)
                # y = 0 (centered vertically)
                # z = target_distance (desired standoff)
                # yaw = 0 (aligned with tag)
                
                # Compute errors in camera frame
                lateral_error = -x           # negative x means tag is to the right, so we need to move right (positive y in body)
                vertical_error = -y          # negative y means tag is down, so we need to move down (positive z in body)
                distance_error = z - self.target_distance
                yaw_error = -math.atan2(x, z)  # tag right (x>0) should produce negative yaw (rotate left)

                # === INDEPENDENT P CONTROLLERS (NO GATING) ===
                # Apply continuous control on all DOFs simultaneously
                
                # Yaw control
                yaw_cmd = self.lock_k_yaw * yaw_error
                yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

                # Forward/backward control (z in body frame)
                forward_cmd = self.lock_k_distance * distance_error
                forward_cmd = max(-self.max_forward_vel, min(self.max_forward_vel, forward_cmd))

                # Left/right control (y in body frame)
                lateral_cmd = self.lock_k_lateral * lateral_error
                lateral_cmd = max(-self.max_lateral_vel, min(self.max_lateral_vel, lateral_cmd))

                # Up/down control (z in body frame - note: in our convention, z is vertical)
                # Map camera vertical error to body vertical velocity
                max_vertical_vel = 0.3  # m/s
                vertical_cmd = self.lock_k_vertical * vertical_error
                vertical_cmd = max(-max_vertical_vel, min(max_vertical_vel, vertical_cmd))

                # Assemble velocity command (body frame convention)
                cmd.linear.x = forward_cmd       # forward/back
                cmd.linear.y = lateral_cmd       # left/right
                cmd.linear.z = vertical_cmd      # up/down
                cmd.angular.z = yaw_cmd         # yaw rotation

                self.get_logger().info(
                    f"LOCK: yaw_err={yaw_error:.3f}rad yaw_cmd={yaw_cmd:.3f}rad/s | "
                    f"dist={z:.2f}m target={self.target_distance:.2f}m fwd_cmd={forward_cmd:.3f}m/s | "
                    f"lat_err={lateral_error:.3f}m lat_cmd={lateral_cmd:.3f}m/s | "
                    f"vert_err={vertical_error:.3f}m vert_cmd={vertical_cmd:.3f}m/s",
                    throttle_duration_sec=0.5
                )

            except TransformException as ex:
                # If we can't get TF, fall back to SEARCH behavior
                self.get_logger().warn(
                    f"LOCK: no valid TF {self.camera_frame}->{self.tag_frame}: {str(ex)}; "
                    f"falling back to SEARCH yaw.",
                    throttle_duration_sec=2.0
                )
                cmd.angular.z = self.search_yaw

        else:
            # SEARCH mode: try to find tag; auto-lock when found
            try:
                # Check if tag is visible
                tf = self._tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.tag_frame,
                    Time()
                )
                
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z
                
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and abs(z) > 1e-3:
                    # Tag found! Use same 4-DOF regulation as LOCK mode
                    lateral_error = -x
                    vertical_error = -y
                    distance_error = z - self.target_distance
                    yaw_error = -math.atan2(x, z)

                    # Apply independent P controllers on all DOFs
                    yaw_cmd = self.lock_k_yaw * yaw_error
                    yaw_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_cmd))

                    forward_cmd = self.lock_k_distance * distance_error
                    forward_cmd = max(-self.max_forward_vel, min(self.max_forward_vel, forward_cmd))

                    lateral_cmd = self.lock_k_lateral * lateral_error
                    lateral_cmd = max(-self.max_lateral_vel, min(self.max_lateral_vel, lateral_cmd))

                    max_vertical_vel = 0.3
                    vertical_cmd = self.lock_k_vertical * vertical_error
                    vertical_cmd = max(-max_vertical_vel, min(max_vertical_vel, vertical_cmd))

                    cmd.linear.x = forward_cmd
                    cmd.linear.y = lateral_cmd
                    cmd.linear.z = vertical_cmd
                    cmd.angular.z = yaw_cmd
                    
                    self.get_logger().info(
                        f"SEARCH->TAG FOUND! Auto-locking. yaw_err={yaw_error:.3f}rad | "
                        f"fwd_cmd={forward_cmd:.3f}m/s | lat_cmd={lateral_cmd:.3f}m/s | vert_cmd={vertical_cmd:.3f}m/s",
                        throttle_duration_sec=0.5
                    )
                else:
                    # TF invalid, continue searching
                    cmd.angular.z = self.search_yaw
            except TransformException:
                # No tag found yet, keep searching
                cmd.angular.z = self.search_yaw

        # Publish debug & actual velocity command
        self._hover_cmd_pub.publish(cmd)
        self._vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HoverYawSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
