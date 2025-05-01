from flask import Flask, render_template, jsonify, request
from quad.nodes.plotter_node import PlotterNode, shared_data, data_lock
from quad.nodes.controller_node import CPGControllerNode  # <--- Import your CPG controller here
from quad.nodes.orb_pose_estimator_node import ORBPosePublisher  # Add this import

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading

app = Flask(__name__, template_folder='templates')

# Global variables to share between threads
plotter_node = None
cpg_controller = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    with data_lock:
        return jsonify(shared_data)

@app.route('/cpg/enable', methods=['POST'])
def cpg_enable():
    state = request.json.get('enable', False)
    if cpg_controller:
        cpg_controller.set_enable(state)
    return jsonify({"success": True})

@app.route('/cpg/gait', methods=['POST'])
def cpg_gait():
    if cpg_controller:
        cpg_controller.change_gait()
        return jsonify({"success": True, "new_gait": cpg_controller.current_gait})
    return jsonify({"success": False, "new_gait": "unknown"})

@app.route('/cpg/params', methods=['POST'])
def cpg_params():
    params = request.json.get('params', {})
    if cpg_controller:
        cpg_controller.update_params(params)
    return jsonify({"success": True})

def ros_spin():
    global plotter_node, cpg_controller

    rclpy.init()

    plotter_node = PlotterNode()
    cpg_controller = CPGControllerNode(plotter_node.servo_pub)
    orb_pose_node = ORBPosePublisher()  # <- Create instance of your pose publisher

    executor = MultiThreadedExecutor()
    executor.add_node(plotter_node)
    executor.add_node(orb_pose_node)  # <- Add it to the executor

    try:
        executor.spin()
    finally:
        plotter_node.destroy_node()
        orb_pose_node.destroy_node()
        rclpy.shutdown()



def main():
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    main()
