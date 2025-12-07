#!/usr/bin/env python3
"""
AprilTag Detector
Detects AprilTags in camera feed and publishes pose information
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

# Try to import apriltag library
try:
    from dt_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    try:
        from pupil_apriltags import Detector
        APRILTAG_AVAILABLE = True
    except ImportError:
        APRILTAG_AVAILABLE = False

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        if not APRILTAG_AVAILABLE:
            self.get_logger().error('AprilTag library not installed!')
            self.get_logger().error('Install with: pip3 install pupil-apriltags')
            self.get_logger().error('Or: pip3 install dt-apriltags')
            raise ImportError('AprilTag library not available')

        # Parameters
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('camera_fx', 600.0)  # Focal length (will auto-detect from camera_info)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)  # Image center
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('tag_size', 0.165)   # meters (6.5 inches)
        self.declare_parameter('debug_image', True)

        self.target_id = self.get_parameter('target_tag_id').value
        self.tag_size = self.get_parameter('tag_size').value
        self.debug_enabled = self.get_parameter('debug_image').value

        # Camera intrinsics (will be updated from camera_info if available)
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]
        self.camera_info_received = False

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, '/apriltag/target_pose', 10)
        self.target_detected_pub = self.create_publisher(Bool, '/apriltag/detected', 10)
        self.target_id_pub = self.create_publisher(Int32, '/apriltag/detected_id', 10)

        if self.debug_enabled:
            self.debug_image_pub = self.create_publisher(Image, '/apriltag/debug_image', 10)

        self.get_logger().info(f'AprilTag Detector started (target ID: {self.target_id})')
        self.get_logger().info(f'  Tag size: {self.tag_size}m ({self.tag_size*39.37:.1f} inches)')

    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera_info topic"""
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_params = [self.fx, self.fy, self.cx, self.cy]
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics updated: fx={self.fx:.1f}, fy={self.fy:.1f}')

    def image_callback(self, msg):
        """Detect AprilTags in camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect tags
            results = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )

            target_found = False
            detected_id = -1

            for result in results:
                # Draw all detected tags
                corners = result.corners.astype(int)
                cv2.polylines(cv_image, [corners], True, (0, 255, 255), 2)
                cv2.putText(
                    cv_image,
                    f'ID {result.tag_id}',
                    (corners[0][0], corners[0][1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2
                )

                # Check if this is our target tag
                if result.tag_id == self.target_id:
                    target_found = True
                    detected_id = result.tag_id

                    # Extract pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera'

                    # Translation (position in camera frame)
                    # AprilTag library returns pose_t as [x, y, z] where:
                    # x = right, y = down, z = forward
                    # Convert to ROS convention: x = forward, y = left, z = up
                    pose_msg.pose.position.x = result.pose_t[2][0]  # Forward (Z in camera)
                    pose_msg.pose.position.y = -result.pose_t[0][0] # Left (-X in camera)
                    pose_msg.pose.position.z = -result.pose_t[1][0] # Up (-Y in camera)

                    # Orientation (for now we just care about position)
                    # Could add quaternion from pose_R if needed
                    pose_msg.pose.orientation.w = 1.0

                    self.target_pose_pub.publish(pose_msg)

                    # Highlight target tag
                    cv2.polylines(cv_image, [corners], True, (0, 255, 0), 3)
                    distance = pose_msg.pose.position.x
                    cv2.putText(
                        cv_image,
                        f'TARGET: {distance:.2f}m',
                        (corners[0][0], corners[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

            # Publish detection status
            self.target_detected_pub.publish(Bool(data=target_found))
            self.target_id_pub.publish(Int32(data=detected_id))

            # Publish debug image
            if self.debug_enabled:
                # Add status text
                status_text = f'Target {self.target_id}: {"FOUND" if target_found else "SEARCHING"}'
                cv2.putText(
                    cv_image,
                    status_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0) if target_found else (0, 0, 255),
                    2
                )
                cv2.putText(
                    cv_image,
                    f'Tags detected: {len(results)}',
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = AprilTagDetector()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
