import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading

class OptiTrackPlotter(Node):
    def __init__(self):
        super().__init__('optitrack_plotter')

        # Subscribe to your VRPN pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/client/RigidBody03/pose',  # << Make sure this matches your topic
            self.listener_callback,
            10)
        
        self.x_data = []
        self.y_data = []
        self.z_data = []

    def listener_callback(self, msg):
        self.x_data.append(msg.pose.position.x)
        self.y_data.append(msg.pose.position.y)
        self.z_data.append(msg.pose.position.z)

def plotting_thread(node):
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    while rclpy.ok():
        ax.clear()
        if node.x_data:
            ax.plot(node.x_data, node.y_data, node.z_data, marker='o')
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.set_zlabel('Z (meters)')
            ax.set_title('RigidBody03 Live Trajectory')
            ax.grid(True)
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackPlotter()

    thread = threading.Thread(target=plotting_thread, args=(node,))
    thread.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
