import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ORBFusionPoseEstimator(Node):
    def __init__(self):
        super().__init__('orb_pose_estimator')

        self.bridge = CvBridge()
        self.prev_frame = None
        self.orb = cv2.ORB_create(1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # State
        self.position = np.zeros(3)  # x, y, z

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Float32, '/ultrasonic/distance', self.ultrasonic_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_estimate', 10)

        self.last_time = None
        self.ultrasonic_distance = None  # In meters

    def ultrasonic_callback(self, msg):
        self.ultrasonic_distance = msg.data / 100.0  # Assuming cm → m

    def image_callback(self, msg):
        curr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        if self.prev_frame is not None:
            prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY)
            kp1, des1 = self.orb.detectAndCompute(prev_gray, None)
            kp2, des2 = self.orb.detectAndCompute(curr_gray, None)

            if des1 is not None and des2 is not None:
                matches = self.bf.match(des1, des2)
                matches = sorted(matches, key=lambda x: x.distance)

                pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 2)
                pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 2)

                if len(pts1) > 5:
                    transform, _ = cv2.estimateAffinePartial2D(pts1, pts2)
                    if transform is not None:
                        dx = transform[0, 2]
                        dy = transform[1, 2]

                        # Scale translation
                        scale = 0.001  # Tune this based on camera setup
                        dt = (self.get_clock().now().nanoseconds * 1e-9) if self.last_time is None else (self.get_clock().now().nanoseconds * 1e-9 - self.last_time)

                        self.position[0] += dx * scale
                        self.position[1] += dy * scale

        # Z correction
        if self.ultrasonic_distance is not None:
            self.position[2] = self.ultrasonic_distance

        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]

        # No orientation for now
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        self.prev_frame = curr_frame
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = ORBFusionPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
