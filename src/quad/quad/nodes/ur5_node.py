#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import socket
import time

ROBOT_IP = "192.168.1.103"
DASHBOARD_PORT = 29999
CMD_TIMEOUT = 3.0

START_PROGRAM = "gripper.urp"
STOP_PROGRAM = "stop.urp"

class URDashboardServiceNode(Node):
    def __init__(self):
        super().__init__('ur_dashboard_service_node')
        self.get_logger().info("UR Dashboard Service Node started.")

        # Create services
        self.start_srv = self.create_service(Trigger, 'start_program', self.handle_start_program)
        self.stop_srv = self.create_service(Trigger, 'stop_program', self.handle_stop_program)

    def send_dashboard(self, cmd: str) -> str:
        try:
            with socket.create_connection((ROBOT_IP, DASHBOARD_PORT), timeout=CMD_TIMEOUT) as sock:
                sock.recv(1024)  # discard banner
                sock.sendall((cmd + "\n").encode("utf-8"))
                time.sleep(0.1)
                return sock.recv(4096).decode("utf-8", errors="ignore").strip()
        except Exception as e:
            self.get_logger().error(f"Dashboard command error: {e}")
            return str(e)

    def load_and_play(self, program_name: str) -> (bool, str):
        self.get_logger().info(f"→ Loading program: {program_name}")
        resp = self.send_dashboard(f"load {program_name}")
        if "Loading program" not in resp:
            return False, f"Failed to load program: {resp}"
        time.sleep(0.3)

        self.get_logger().info("→ Starting program...")
        resp = self.send_dashboard("play")
        if any(k in resp for k in ("Starting program", "Running program")):
            return True, f"Program started: {resp}"
        return False, f"Failed to start program: {resp}"

    def handle_start_program(self, request, response):
        success, message = self.load_and_play(START_PROGRAM)
        response.success = success
        response.message = message
        return response

    def handle_stop_program(self, request, response):
        success, message = self.load_and_play(STOP_PROGRAM)
        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = URDashboardServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

