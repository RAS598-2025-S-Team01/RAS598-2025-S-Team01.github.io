# Fusion Robotics: Quadruped-UR5 Autonomous Testbed (Team 01)

[![ROS Version](https://img.shields.io/badge/ROS%202-Humble-blueviolet)](https://docs.ros.org/en/humble/index.html)
[![Flask API](https://img.shields.io/badge/Flask-Humble-blue)](https://docs.ros.org/en/humble/index.html)

<!-- Add your LICENSE file -->

**Course:** RAS598 S2025, Arizona State University
**Team:** Team 01

## 1. Overview

Fusion Robotics establishes an autonomous testbed integrating a quadruped robot and a UR5 robotic arm, orchestrated using ROS 2 Humble. The core objective is to create a repeatable, automated workflow for testing quadruped locomotion, collecting diverse sensor data (IMU, Camera, Servo Encoders), evaluating performance through comparative pose estimation, and enabling real-time control via a web-based user interface.

This project prioritizes the seamless integration and interaction between the UR5 and the quadruped for an automated reset sequence, providing a stable platform for robotics experimentation and data acquisition. The UR5 arm is controlled directly via **WebSockets and the UR Dashboard API** for pick-and-place operations.

## 2. Key Features

*   **Autonomous Workflow:** Fully automated cycle: UR5 places the quadruped -> Quadruped walks -> Sensor data is logged -> Quadruped stops -> UR5 retrieves and resets the quadruped.
*   **Real-time Web UI Control:** A Flask-based web interface (`/ros2_flask_plotter`) allows:
    *   Real-time visualization of sensor data (IMU, Pose Estimates, Servo State).
    *   Interactive control over quadruped gait parameters (Frequency, Duty Factor, Offsets, Amplitudes).
    *   Selection of different movement modes (Walk, Trot, Pronk, etc.).
*   **Multi-Sensor Data Acquisition:** Collects and logs data from:
    *   Inertial Measurement Unit (IMU)
    *   Camera (for visual pose estimation)
    *   Servo Encoders (via `/bridge_node`)
*   **Comparative Pose Estimation:** Implements and logs pose estimates from:
    *   IMU-based filtering (`/pose_estimator_node` -> `/imu/pose_estimate`)
    *   Camera-based visual methods (`/optical_flow_pose_publisher` -> `/orb/pose`)
    *   Allows for direct comparison and validation via the UI.
*   **Modular ROS 2 Architecture:** Leverages ROS 2 Humble for robust communication between hardware, estimation nodes, control logic, and the UI.
*   **UR5 Control via WebSockets/Dashboard API:** Direct control of the UR5 arm for pick-and-place tasks using its native network interfaces.
*   **Open Resources:** Aims to share developed ROS 2 packages and setup instructions.

## 3. System Architecture

The system utilizes ROS 2 Humble for modular communication.

<!-- TODO: If you have the diagram image hosted, embed it here -->
<!-- ![ROS 2 Architecture Diagram](./path/to/your/diagram.png) -->
*Figure: Finalized ROS 2 Node and Topic Architecture (See project description for details)*

### 3.1 Key ROS 2 Nodes

*   **/bridge_node**: Interfaces directly with the quadruped's hardware (servos). Corresponds to the `connect` executable, implemented in `nodes/controller_node.py` and/or `nodes/load_esp32.py`.
    *   *Subscribes*: `/servo/command`, Parameter updates (for gait control).
    *   *Publishes*: `/servo/state`.
    *   *Function*: Handles low-level servo communication and gait generation based on current parameters.
*   **/pose_estimator_node**: Performs IMU-based pose estimation. Implemented in `nodes/pose_estimator_node.py`.
    *   *Subscribes*: `/imu/data`, `/imu/orientation`.
    *   *Publishes*: `/imu/pose_estimate`.
    *   *Function*: Integrates IMU data (using Kalman/complementary filters) to estimate orientation and relative position.
*   **/optical_flow_pose_publisher** (or similar): Performs camera-based pose estimation. Implemented in `nodes/orb_pose_estimator_node.py`.
    *   *Subscribes*: (Implied) Camera image topic.
    *   *Publishes*: `/orb/pose`.
    *   *Function*: Processes visual features or optical flow to estimate pose relative to the environment.
*   **/ros2_flask_plotter**: Acts as the bridge between ROS 2 and the web UI. Corresponds to the `app` executable, implemented in `web/app.py`.
    *   *Subscribes*: `/imu/data`, `/imu/orientation`, `/imu/pose_estimate`, `/orb/pose`, `/servo/state`, `/parameter_events`.
    *   *Publishes/Services*: Handles UI commands to change parameters or trigger actions.
    *   *Function*: Runs a Flask web server to serve the UI (`web/templates/index.html`), visualizes subscribed data, and relays user commands back into the ROS 2 system.
*   **UR5 Interface Node**: Orchestrates UR5 arm control. Corresponds to the `ur5` executable, implemented in `nodes/ur5_node.py`.
    *   *Function*: Communicates with the UR5 controller via **WebSockets and the UR Dashboard API** to execute pre-programmed pick-and-place routines. It may subscribe to ROS 2 topics or services to receive triggers for these actions.
*   **(Implied Nodes)**:
    *   Sensor driver nodes publishing raw data (e.g., `/imu/data`, `/imu/orientation`, camera data). These are external packages or custom nodes not shown in the `tree`.

### 3.2 Key ROS 2 Topics

The following ROS 2 topics facilitate communication between the nodes:

*   `/imu/data`: Raw IMU accelerometer/gyroscope data.
*   `/imu/orientation`: Filtered IMU orientation (e.g., quaternions).
*   `/imu/pose_estimate`: Calculated pose from IMU data (by `/pose_estimator_node`).
*   `/orb/pose`: Calculated pose from camera/visual data (by `/optical_flow_pose_publisher`).
*   `/servo/command`: Low-level commands sent to quadruped servos (generated by `/bridge_node`).
*   `/servo/state`: Current state feedback from servos (published by `/bridge_node`).
*   `/parameter_events`: Standard ROS 2 topic for broadcasting parameter changes across nodes.
*   `/rosout`: Standard ROS 2 logging topic.

### 3.3 Quadruped Control Parameters (via UI)

| Parameter | Symbol | Description                                       |
| :-------- | :----- | :------------------------------------------------ |
| Frequency | F      | Oscillation frequency (controls step speed)       |
| Duty Factor | DF     | Portion of cycle foot is on the ground          |
| Foot Offset | FO     | Lateral position offset for feet                |
| Elbow Offset| EO     | Adjusts the base elbow joint position offset    |
| Hip Amplitude| HA     | Range of motion for the hip joint               |
| Hip Offset  | HO     | Base angle shift for the hip joint              |
| Knee Amplitude| KA     | Range of motion for the knee joint              |
| Knee Offset | KO     | Base angle shift for the knee joint             |
| Movement Mode | -    | Gait selection: PRONK, WALK, PACE, TROT, BOUND |

### 3.4 Example Data Flow (UI Control)

1.  User changes 'Frequency' (F) on the Web UI.
2.  UI sends the new value via the Flask backend within `/ros2_flask_plotter` (`app` node, `web/app.py`).
3.  `/ros2_flask_plotter` updates the 'F' parameter on the `/bridge_node` (`connect` node) using a ROS 2 parameter service call.
4.  `/bridge_node` detects the parameter change via its callback.
5.  `/bridge_node` adjusts its internal gait generation logic based on the new 'F' value.
6.  `/bridge_node` publishes updated joint targets on `/servo/command`.
7.  Quadruped hardware responds, changing its walking speed.
8.  Resulting changes in motion affect sensor readings (`/imu/*`, `/orb/pose`), which are published by respective nodes.
9.  `/ros2_flask_plotter` receives the updated sensor data and refreshes the plots on the Web UI.

## 4. Core Autonomous Workflow

The system executes the following automated cycle for data collection:

1.  **Initialization:** The UR5 arm (controlled by the `ur5` node via WebSocket/Dashboard API) precisely places the quadruped robot at a designated starting position.
2.  **Autonomous Locomotion:** Triggered via the ROS 2 system (potentially through the UI via the `app` node), the quadruped (controlled by the `connect` node) begins walking forward based on the set gait parameters.
3.  **Real-time Sensing:** Onboard sensors (IMU, Camera, Servo Encoders) continuously publish data to their respective ROS 2 topics.
4.  **Data Logging & Visualization:** The `/ros2_flask_plotter` (`app` node, `web/app.py`) subscribes to sensor and pose topics, logging the data and visualizing key metrics in real-time on the web interface.
5.  **Path Completion & Reset Trigger:** Upon reaching a predefined distance or condition (monitored via `/imu/pose_estimate` or `/orb/pose`), the quadruped stops. A signal is sent to the `ur5` node (e.g., via a ROS 2 topic or service).
6.  **Automated Reset:** The `ur5` node receives the trigger and commands the UR5 arm via WebSocket/Dashboard API to execute a pre-programmed routine to gently pick up the quadruped and return it to the starting position.
7.  **Iteration:** The cycle repeats for continuous, automated data collection.

## 5. Technical Considerations & Tradeoffs

Developing this system involved balancing several factors:

*   **Pose Estimation (Accuracy vs. Resources):** Implemented parallel IMU (`nodes/pose_estimator_node.py`) and Camera (`nodes/orb_pose_estimator_node.py`) estimators instead of computationally heavy real-time fusion, allowing comparison via the UI.
*   **Real-time Control vs. Stability:** Gait parameter changes from the UI are rate-limited or smoothed by the `/bridge_node` (`connect` node) to prevent dynamically unstable commands and ensure robot safety.
*   **Hardware Constraints vs. Algorithm Choice:** Focused on reliable core functionality using efficient algorithms suitable for potentially limited onboard compute, deferring resource-intensive features like full SLAM.
*   **Development Time vs. Feature Scope:** Prioritized delivering the core Minimum Viable System (automated loop, UI control, data pipeline) over deeper simulation integration or advanced fusion within the initial timeframe.
*   **Path End Detection (Robustness vs. Precision):** Employs conservative stopping logic and a generous UR5 pickup envelope to ensure high reliability of the automated reset cycle, potentially at the cost of exact distance traveled per run.
*   **UR5 Control Method:** Using WebSockets/Dashboard API allows direct access to UR-specific commands but bypasses some standard ROS 2 control frameworks (like MoveIt 2) for the pick-and-place actions, potentially simplifying the setup for this specific task but requiring custom implementation for motion planning and execution within the `ur5` node.

## 6. Folder Structure

The `quad` ROS 2 package follows this structure:


```bash
quad/
├── package.xml             # ROS 2 package manifest (dependencies, metadata)
├── quad/                   # Main Python module source directory
│   ├── init.py         # Makes 'quad' a Python package
│   ├── nodes/              # Contains Python scripts defining ROS 2 nodes
│   │   ├── init.py
│   │   ├── controller_node.py # Handles quadruped control logic/gait generation
│   │   ├── load_esp32.py      # Node for interfacing with an ESP32 microcontroller
│   │   ├── orb_pose_estimator_node.py # Node for ORB/Camera-based pose estimation
│   │   ├── plotter_node.py    # Node for plotting/visualizing data (part of UI backend)
│   │   ├── pose_estimator_node.py # Node for IMU-based pose estimation
│   │   ├── ur5_node.py        # Node for controlling the UR5 arm
│   │   └── pycache/
│   ├── scripts/            # Contains utility scripts (may or may not be ROS nodes)
│   │   ├── cpg_controller.py  # Central Pattern Generator implementation
│   │   ├── optitrack.py       # Script related to OptiTrack motion capture system
│   │   ├── orb_pose_estimation.py # Supporting script for ORB pose estimation
│   │   └── pycache/
│   ├── web/                # Contains files for the Flask web application (UI)
│   │   ├── init.py
│   │   ├── app.py           # Main Flask application script (ROS node 'app')
│   │   ├── templates/       # HTML templates for the web UI
│   │   │   └── index.html
│   │   └── pycache/
│   └── pycache/        # Compiled Python files for the 'quad' module
├── resource/               # ROS 2 resource marker directory
│   └── quad                # Empty package marker file
├── setup.cfg               # Configuration for setuptools
├── setup.py                # Python package setup script. Defines ROS 2 executables ('app', 'connect', 'ur5') and maps them to entry points within the Python scripts.
└── test/                   # Contains package tests
├── test_copyright.py
├── test_flake8.py
└── test_pep257.py
```

**Key Points from Structure:**

*   **Node Definitions:** Core ROS 2 nodes are defined in individual files within the `quad/nodes/` directory.
*   **Web Application:** The Flask web UI code is organized under `quad/web/`, with `app.py` being the main entry point integrating Flask and ROS 2 for the UI node.
*   **Executables:** The `setup.py` file defines the `app`, `connect`, and `ur5` executables, specifying their entry points (e.g., a `main` function within `quad.web.app`, `quad.nodes.ur5_node`, etc.). The `connect` executable maps to the node(s) responsible for hardware interface/control (`controller_node.py` and/or `load_esp32.py`). The `ur5` executable maps to `nodes/ur5_node.py` which handles WebSocket/Dashboard API communication.
*   **Dependencies:** `package.xml` declares all necessary ROS, system, and Python dependencies (`rclpy`, `Flask`, sensor message types, WebSocket/HTTP libraries, etc.).

## 7. Prerequisites

*   **ROS 2 Humble Hawksbill:** Installed and configured.
*   **Colcon:** Standard ROS 2 build tool (`sudo apt install python3-colcon-common-extensions`).
*   **Git:** (`sudo apt install git`).
*   **Python 3 & Pip:** Required for ROS 2 nodes and Flask.
*   **Flask:** Python web framework (`pip install Flask`). Should be installed via `rosdep` if listed correctly in `package.xml`.
*   **Python WebSocket/HTTP Libraries:** Libraries needed by `nodes/ur5_node.py` to communicate with the UR5 (e.g., `websockets`, `requests`). Ensure these are installed (e.g., `pip install websockets requests`) or listed in `package.xml`.
*   **rosdep:** (`sudo apt install python3-rosdep`, then `sudo rosdep init`, `rosdep update`).
*   **UR5 Network Configuration:** Ensure the UR5 controller is accessible on the network from the computer running the ROS nodes and that its Dashboard Server and relevant interfaces are enabled.
*   **Quadruped Specific Drivers/Interfaces:** Any necessary low-level drivers or libraries for communicating with the quadruped hardware if not handled entirely by the `connect` node.
*   **OpenCV:** Required for `orb_pose_estimator_node.py`.

## 8. Installation and Building

1.  **Create/Navigate to ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/
    cd ~/ros2_ws
    ```

2.  **Clone Repository:**
    Assuming the `quad` package shown in the tree is directly within the `project_code` repository:
    ```bash
    # Clone the main repository into your workspace's src directory
    git clone https://github.com/RAS598-2025-S-Team01/RAS598-2025-S-Team01-code ~/ros2_ws
    ```
    *Adjust the clone command if the `quad` package is at the root of the repository.*

3.  **Install Dependencies:**
    Navigate to the workspace root (`~/ros2_ws`) and install dependencies listed in `package.xml` files within the `src` directory.
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
    *This command will install system dependencies. Ensure Python dependencies like Flask and the WebSocket/HTTP libraries are correctly listed in `package.xml`'s `<exec_depend>` tags or install them manually via pip (`pip install Flask websockets requests ...`).*

4.  **Build the Workspace:**
    Source your base ROS 2 installation and build the packages.
    ```bash
    # Replace 'humble' if using a different ROS 2 version
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

## 9. Running the System

To run the full system, you need to launch the core components concurrently. This involves running each command in a separate terminal. Ensure you have sourced your workspace in each terminal before running the commands.

1.  **Terminal 1: Launch Quadruped Interface (`connect` node)**
    *   Source your workspace:
        ```bash
        cd ~/ros2_ws
        source install/setup.bash
        ```
    *   Run the node responsible for quadruped hardware communication and gait generation (`/bridge_node`):
        ```bash
        ros2 run quad connect
        ```
    *   Keep this terminal running. It handles the low-level control and state feedback of the quadruped.

2.  **Terminal 2: Launch UR5 Interface (`ur5` node)**
    *   Open a *new* terminal.
    *   Source your workspace:
        ```bash
        cd ~/ros2_ws
        source install/setup.bash
        ```
    *   Run the node responsible for interfacing with the UR5 arm via WebSocket/Dashboard API:
        ```bash
        ros2 run quad ur5
        ```
    *   Keep this terminal running. It manages the UR5's actions based on triggers from the ROS system.

3.  **Terminal 3: Launch Web UI (`app` node)**
    *   Open a *new* terminal.
    *   Source your workspace:
        ```bash
        cd ~/ros2_ws
        source install/setup.bash
        ```
    *   Run the Flask application node (`/ros2_flask_plotter`):
        ```bash
        ros2 run quad app
        ```
    *   Look for output indicating the Flask server is running, typically like:
        `* Running on http://127.0.0.1:5000/` or `http://0.0.0.0:5000/`. Note the URL.
    *   Keep this terminal running. It serves the web interface and bridges UI interactions with ROS 2.
    *   Verify ros2 topics using the command:
        ```bash
        ros2 topic list
        ```

4.  **Access Web Interface:**
    *   Open a web browser on a device connected to the same network as the computer running the nodes.
    *   Navigate to the URL provided in Terminal 3 (e.g., `http://<your_robot_ip_or_127.0.0.1>:5000`).
    *   Interact with the UI to monitor data and control the quadruped.

*(Optional: Consider creating a ROS 2 launch file (`.launch.py`) within your `quad` package to start all three nodes (`connect`, `ur5`, `app`) and potentially the pose estimation nodes with a single command for convenience.)*

## 10. Validation & Success Metrics

Project success is measured by:

*   **Workflow Completion:** Successfully executing multiple consecutive autonomous walk-and-reset cycles.
*   **Pose Estimation Accuracy:** Qualitative and quantitative comparison of `/imu/pose_estimate` and `/orb/pose` via UI plots and logged data (assessing consistency, drift).
*   **Control Responsiveness:** Demonstrating immediate and observable changes in quadruped locomotion corresponding to real-time parameter adjustments via the web UI.
*   **Data Integrity:** Ensuring logged sensor data is consistent, correctly timestamped, and accurately reflects the robot's state.

## 11. Future Work

Potential areas for future development include:

*   Implementing robust sensor fusion (e.g., EKF) to combine IMU and camera pose estimates into a single, more accurate state estimate topic.
*   Integrating simulation environments (e.g., Gazebo, Isaac Sim) for testing control algorithms before hardware deployment.
*   Leveraging OptiTrack data (using `scripts/optitrack.py`) for ground truth comparison or real-time correction.
*   Developing more advanced SLAM capabilities for mapping and localization.
*   Refining path planning and obstacle avoidance for the quadruped.
*   Adding more sophisticated data analysis tools accessible via the web UI or offline scripts.
*   Potentially integrating standard ROS 2 control interfaces (like MoveIt 2) for more complex UR5 tasks beyond the current pick-and-place routine.

## 12. Acknowledgments

* This project was developed for the RAS598 course (Spring 2025) at Arizona State University.
* Special thanks to Dr. Daniel Aukes for his invaluable guidance and for providing the resources necessary for this project.
* Team 01 Members:
    * Jeevan Hebbal Manjunath
    * Yeshwanth Reddy Gurreddy
    * Aniruddha Anand Damle

---

