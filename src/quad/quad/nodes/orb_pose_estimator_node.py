import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import requests
from collections import deque
from scipy.signal import savgol_filter

VIDEO_URL = 'http://100.81.43.28:5000/video_feed'
ULTRASONIC_URL = 'http://100.81.43.28:5000/ultrasonic_data'

camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0,   0,   1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

class ORBPosePublisher(Node):
    def __init__(self):
        super().__init__('optical_flow_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/orb/pose', 10)

        self.scale_z = 1.0
        self.prev_gray = None
        self.prev_pts = None
        self.initialized = False
        self.pose_R = np.eye(3)
        self.pose_t = np.zeros((3, 1))

        self.buffer_size = 11  # Should be odd for savgol_filter
        self.pose_buffer = deque(maxlen=self.buffer_size)

        self.cap = cv2.VideoCapture(VIDEO_URL)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream.")
            exit(1)

        self.set_initial_reference()
        self.timer = self.create_timer(0.1, self.process_frame)

    def fetch_ultrasonic_distance(self):
        try:
            response = requests.get(ULTRASONIC_URL, timeout=1)
            if response.status_code == 200:
                data = response.json()
                distance = float(data.get("distance_cm", 100.0))
                return distance / 100.0 if distance != -1 else None
        except:
            pass
        return None

    def set_initial_reference(self):
        self.get_logger().info("Capturing initial reference frame...")
        distance = self.fetch_ultrasonic_distance()
        if distance:
            self.scale_z = distance

        while not self.initialized:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.rotate(frame, cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pts = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.01, minDistance=7)

            if pts is not None and len(pts) >= 8:
                self.prev_gray = gray
                self.prev_pts = pts
                self.initialized = True
                self.get_logger().info(f"Initial keypoints: {len(pts)}")

    def process_frame(self):
        if not self.initialized:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None)
        good_prev = self.prev_pts[status.flatten() == 1]
        good_next = next_pts[status.flatten() == 1]

        if len(good_prev) < 8 or len(good_next) < 8:
            return

        E, mask = cv2.findEssentialMat(good_next, good_prev, camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return

        _, R, t, mask_pose = cv2.recoverPose(E, good_next, good_prev, camera_matrix)

        # Accumulate pose
        self.pose_t += self.pose_R @ t * self.scale_z
        self.pose_R = R @ self.pose_R

        # Update buffer
        self.pose_buffer.append(self.pose_t.copy())

        # Apply smoothing if buffer full
        if len(self.pose_buffer) < self.buffer_size:
            smoothed_t = self.pose_t
        else:
            arr = np.array(self.pose_buffer).squeeze()  # shape: (N, 3)
            smoothed = savgol_filter(arr, window_length=self.buffer_size, polyorder=2, axis=0)
            smoothed_t = smoothed[-1].reshape(3, 1)

        # Publish smoothed pose
        quat = self.rotation_matrix_to_quaternion(self.pose_R)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        msg.pose.position.x = float(smoothed_t[0])
        msg.pose.position.y = float(smoothed_t[1])
        msg.pose.position.z = float(smoothed_t[2])
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.publisher_.publish(msg)

        self.prev_gray = gray
        self.prev_pts = good_next.reshape(-1, 1, 2)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = np.empty((4,), dtype=np.float64)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q[[0, 1, 2, 3]]

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
