import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np

from tf_transformations import quaternion_from_euler

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        # Subscribers
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/imu/pose_estimate',
            10
        )
        self.orientation_publisher = self.create_publisher(
            Quaternion,
            '/imu/orientation',
            10
        )

        # State Variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

        self.last_time = None

        # Calibration
        self.bias_accel = np.zeros(3)
        self.bias_gyro = np.zeros(3)
        self.calibration_data_accel = []
        self.calibration_data_gyro = []
        self.calibrated = False
        self.calibration_start_time = None

        # Filtering
        self.alpha = 0.8
        self.filtered_accel = np.zeros(3)
        self.filtered_gyro = np.zeros(3)
        self.velocity_damping = 0.01

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Start calibration timer
        if self.calibration_start_time is None:
            self.calibration_start_time = current_time

        if not self.calibrated:
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            self.calibration_data_accel.append(accel)
            self.calibration_data_gyro.append(gyro)

            if (current_time - self.calibration_start_time) >= 1.0:
                self.bias_accel = np.mean(self.calibration_data_accel, axis=0)
                self.bias_gyro = np.mean(self.calibration_data_gyro, axis=0)
                self.calibrated = True
                self.get_logger().info(f"Calibration complete. Bias Accel: {self.bias_accel}, Bias Gyro: {self.bias_gyro}")
            return

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # Correct readings
        raw_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]) - self.bias_accel

        raw_gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]) - self.bias_gyro

        # Filtering - Low-pass filter
        self.filtered_accel = self.alpha * self.filtered_accel + (1 - self.alpha) * raw_accel
        self.filtered_gyro = self.alpha * self.filtered_gyro + (1 - self.alpha) * raw_gyro

        # Integrate
        self.velocity += self.filtered_accel * dt

        # High-pass filter (remove slow drift in velocity)
        velocity_decay = np.exp(-0.5 * dt)  # decay factor based on time step
        self.velocity *= velocity_decay

        # Small velocity threshold to stop drift when stationary
        velocity_threshold = 0.02  # meters/second
        self.velocity[np.abs(self.velocity) < velocity_threshold] = 0.0

        # Update position
        self.position += self.velocity * dt

        # Integrate orientation
        self.orientation += self.filtered_gyro * dt

        # Create PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])

        # Convert orientation
        q = quaternion_from_euler(
            self.orientation[0],
            self.orientation[1],
            self.orientation[2]
        )

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # Publish pose
        self.pose_publisher.publish(pose_msg)

        # Publish orientation separately
        q_msg = Quaternion()
        q_msg.x = q[0]
        q_msg.y = q[1]
        q_msg.z = q[2]
        q_msg.w = q[3]
        self.orientation_publisher.publish(q_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
