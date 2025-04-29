import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np
import serial
import socket
import time
import threading
from rclpy.executors import MultiThreadedExecutor

from quad.nodes.plotter_node import PlotterNode
from quad.nodes.pose_estimator_node import PoseEstimatorNode
from quad.nodes.controller_node import CPGControllerNode

class Bridge:
    ESP_IP = '192.168.4.1'
    PC_IP = '192.168.4.2'
    PORT = 50000

    LENGTH = 60
    HEADER = (0x01).to_bytes(1, 'big')
    FOOTER = (0x02).to_bytes(1, 'big')

    ENABLE_ALL_SREVOS = [1] * 8
    DISABLE_ALL_SREVOS = [0] * 8
    RESET_POS = [0] * 8

    TX_DATA_TYPE = [
        'en',
        'pos0', 'pos1', 'pos2', 'pos3', 'pos4', 'pos5', 'pos6', 'pos7',
    ]
    RX_DATA_TYPE = [
        't', 'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz', 'ax', 'ay', 'az',
        'pos0', 'pos1', 'pos2', 'pos3', 'pos4', 'pos5', 'pos6', 'pos7',
        'cur0', 'cur1', 'cur2', 'cur3', 'cur4', 'cur5', 'cur6', 'cur7'
    ]

    def __init__(self, port="/dev/ttyACM0"):
        self.port = port
        self.serial = None
        self.socket = None
        self.tx_data = np.zeros(len(self.TX_DATA_TYPE))
        self.rx_data = np.zeros(len(self.RX_DATA_TYPE))
        self.set_tx_data(self.DISABLE_ALL_SREVOS, self.RESET_POS)

    def set_tx_data(self, en, pos):
        self.tx_data[0] = int(''.join([str(v) for v in en]), 2)
        self.tx_data[1:1 + len(pos)] = np.array(pos)

        self.tx_bytes = bytes()
        self.tx_bytes = self.tx_bytes.join(
            [self.HEADER] +
            [int(self.tx_data[0]).to_bytes(1, 'big')] +
            [int(val * 1000).to_bytes(2, 'big', signed=True) for val in self.tx_data[1:]] +
            [(0).to_bytes(self.LENGTH - 1 - 16, 'big')] +
            [self.FOOTER]
        )

    def set_rx_data(self):
        rx_bytes = self.rx_bytes[1:-1]
        self.rx_data[0] = int.from_bytes(rx_bytes[0:4], 'big') / 1e6

        for i in range(4):
            self.rx_data[1 + i] = int.from_bytes(rx_bytes[4 + i * 2:6 + i * 2], 'big', signed=True) / 2**14
        for i in range(3):
            self.rx_data[5 + i] = int.from_bytes(rx_bytes[12 + i * 2:14 + i * 2], 'big', signed=True) / 16 / 180 * np.pi
        for i in range(3):
            self.rx_data[8 + i] = int.from_bytes(rx_bytes[18 + i * 2:20 + i * 2], 'big', signed=True) / 100
        for i in range(8):
            self.rx_data[11 + i] = int.from_bytes(rx_bytes[24 + i * 2:26 + i * 2], 'big', signed=True) / 1000
        for i in range(8):
            self.rx_data[19 + i] = int.from_bytes(rx_bytes[40 + i * 2:42 + i * 2], 'big', signed=True) / 1000

    def tx_rx(self):
        if self.port is None:
            if self.socket is None:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.settimeout(1)
                self.socket.bind((self.PC_IP, self.PORT))
            self.socket.sendto(self.tx_bytes, (self.ESP_IP, self.PORT))
            self.rx_bytes = self.socket.recvfrom(self.LENGTH + 2)[0]
        else:
            if self.serial is None:
                self.serial = serial.Serial(self.port, baudrate=2000000, timeout=1)
            self.serial.write(self.tx_bytes)
            self.rx_bytes = self.serial.read(self.LENGTH + 2)

        if len(self.rx_bytes) != self.LENGTH + 2:
            return False
        if self.rx_bytes[0] != self.HEADER[0] or self.rx_bytes[-1] != self.FOOTER[0]:
            return False

        self.set_rx_data()
        return True


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        self.get_logger().info('Connected to ESP32')

        self.bridge = Bridge()
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, 'servo/state', 10)
        self.create_subscription(Float64MultiArray, 'servo/command', self.command_callback, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        if not self.bridge.tx_rx():
            self.get_logger().warning('Failed to receive data from bridge')
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation = Quaternion(
            w=float(self.bridge.rx_data[1]),
            x=float(self.bridge.rx_data[2]),
            y=float(self.bridge.rx_data[3]),
            z=float(self.bridge.rx_data[4])
        )
        imu_msg.angular_velocity = Vector3(
            x=float(self.bridge.rx_data[5]),
            y=float(self.bridge.rx_data[6]),
            z=float(self.bridge.rx_data[7])
        )
        imu_msg.linear_acceleration = Vector3(
            x=float(self.bridge.rx_data[8]),
            y=float(self.bridge.rx_data[9]),
            z=float(self.bridge.rx_data[10])
        )
        self.imu_pub.publish(imu_msg)

        servo_msg = Float64MultiArray()
        servo_msg.data = list(self.bridge.rx_data[11:19])
        self.servo_pub.publish(servo_msg)

    def command_callback(self, msg):
        pos = msg.data

        # Check if all positions are zero
        if all(abs(p) < 1e-6 for p in pos):  # small tolerance
            # Disable all servos + reset position
            self.get_logger().info('Reset command received: Disabling all servos.')
            self.bridge.set_tx_data(self.bridge.DISABLE_ALL_SREVOS, [0.0] * 8)
        else:
            # Enable servos and send normal positions
            self.bridge.set_tx_data(self.bridge.ENABLE_ALL_SREVOS, pos)


def main(args=None):
    rclpy.init()

    bridge_node = BridgeNode()
    plotter_node = PlotterNode()
    pose_estimator_node = PoseEstimatorNode()

    # This is NOT added to executor
    cpg_controller = CPGControllerNode(plotter_node.servo_pub)

    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(plotter_node)
    executor.add_node(pose_estimator_node)

    executor.spin()

    bridge_node.destroy_node()
    plotter_node.destroy_node()
    pose_estimator_node.destroy_node()

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

