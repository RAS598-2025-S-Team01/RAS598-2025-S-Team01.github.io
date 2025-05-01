# === plotter_node.py ===
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion

import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import threading

MAX_HISTORY = 201
shared_data = {
    'time': [],
    'servo': [],
    'euler': [],
    'gyro': [],
    'accel': [],
    'pose': [],
    'orientation_quat': [],
    'slam_pose': [],
    'slam_orientation_quat': [],
}
data_lock = threading.Lock()

class PlotterNode(Node):
    def __init__(self):
        super().__init__('ros2_flask_plotter')

        self.servo_data = np.zeros(8)
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)
        self.latest_pose = np.zeros(3)
        self.orientation_quat = np.zeros(4)
        self.latest_slam_pose = np.zeros(3)
        self.latest_slam_orientation = np.zeros(4)

        self.servo_pub = self.create_publisher(Float64MultiArray, 'servo/command', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, 'imu/data', self.imu_callback, sensor_qos)
        self.create_subscription(Float64MultiArray, 'servo/state', self.servo_callback, 10)
        self.create_subscription(PoseStamped, 'imu/pose_estimate', self.pose_callback, 10)
        self.create_subscription(Quaternion, '/imu/orientation', self.orientation_callback, 10)
        self.create_subscription(PoseStamped, '/orb/pose', self.slam_pose_callback, 10)

        self.create_timer(0.02, self.sync_callback)

    def imu_callback(self, msg: Imu):
        self.orientation = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def servo_callback(self, msg: Float64MultiArray):
        self.servo_data = np.array(msg.data)

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def orientation_callback(self, msg: Quaternion):
        self.orientation_quat = np.array([msg.w, msg.x, msg.y, msg.z])

    def slam_pose_callback(self, msg: PoseStamped):
        self.latest_slam_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.latest_slam_orientation = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

    def sync_callback(self):
        now = time.time()
        try:
            r = R.from_quat([self.orientation[1], self.orientation[2], self.orientation[3], self.orientation[0]])
            euler = r.as_euler('ZYX', degrees=False)[::-1]
        except Exception as e:
            self.get_logger().warn(f'Quaternion conversion failed: {e}')
            euler = [np.nan, np.nan, np.nan]

        with data_lock:
            shared_data['time'].append(now)
            shared_data['servo'].append(self.servo_data.tolist())
            shared_data['euler'].append(euler.tolist())
            shared_data['gyro'].append(self.gyro.tolist())
            shared_data['accel'].append(self.accel.tolist())
            shared_data['pose'].append(self.latest_pose.tolist())
            shared_data['orientation_quat'].append(self.orientation_quat.tolist())
            shared_data['slam_pose'].append(self.latest_slam_pose.tolist())
            shared_data['slam_orientation_quat'].append(self.latest_slam_orientation.tolist())

            if len(shared_data['time']) > MAX_HISTORY:
                for k in shared_data:
                    if shared_data[k] and len(shared_data[k]) > MAX_HISTORY:
                        shared_data[k].pop(0)