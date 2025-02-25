# Multi-Agent-Control-Quadruped-Meets-UR5

## **Project:** Fusion Robotics: A Quadruped-UR5 Robotics Testbed

**Team Number:**  
Team 01

**Team Members:**  

1. Jeevan Hebbal Manjunath
1. Yeshwanth Reddy Gurreddy
1. Aniruddha Anand Damle

**Semester and Year:**  
Spring 2025

**University, Class, and Professor:**  
- **University:** Arizona State University  
- **Class:** Experimentation and Deployment of Robotic Systems  
- **Professor:** Dr. Daniel M. Aukes

---

## 3. Project Plan

### 3.1 High-Level Concept and Research Question

Our project aims to co-ordinate a UR5 robotic arm with a quadruped robot to facilitate advanced experimental setups. The central research question is: **"How can the integration of sensor data and simulation models optimize the accuracy and efficiency of experimental setups in laboratories"**  
The experiment involves using the UR5 to position the quadruped at a designated start point, triggering the quadruped to run in a specified direction, and collecting sensor data during its motion. We then broadcast this data to a server for analysis, comparing it with simulated data. The refined simulation results are subsequently applied back to the robot to evaluate improvements in performance forming a closed loop.

> ![High-Level System Concept](./static/images/quadraped.png)  
> **Figure 1: Illustrates the the design of quadruped, which is used for sensor data collection.**

### 3.2 Sensor Integration

Sensor integration is critical to our experimental setup. We plan to collect a range of sensor data (e.g., IMU, Ultrasonic, joint encoders, Camera) during the quadruped’s run. This data will be processed and filtered in real time to remove noise and extract meaningful features. In testing, sensors will validate both the starting position and the dynamic state of the quadruped. In the final demonstration, real-time sensor feedback will be used to adjust simulation parameters and fine-tune control algorithms, ensuring the robot operates optimally even under environmental variations.

### 3.3 Interaction and Interface Development

Interaction with the system will be two-fold:  

1. **Visualization & Monitoring:**  
   A user-friendly interface will display real-time sensor data, simulation comparisons, and status of hardware integration. This interface will allow operators to observe and intervene if necessary.  
2. **Data Storage and Interaction:**  
   The system will log all sensor data and simulation outputs for post-experiment analysis. This will enable retrospective analysis and adjustments to the simulation model.
 
> ![Interface Mockup](./static/images/proposed_experimentaion.png)  
> **Figure 2: Experimental setup rendering.**

### 3.4 Control and Autonomy

To enable efficient data collection and autonomous operation, we will implement a **sensor-driven control loop** that dynamically adjusts the quadruped’s trajectory based on real-time feedback. The control system consists of two layers:

#### 3.4.1 **Control Architecture**  

- **Low-Level Control:**  
	- Regulates the quadruped’s locomotion using real-time sensor input.  
	- Ensures stability and responsiveness as it traverses the path while collecting data.  

- **High-Level Decision Making:**  
	- Oversees the overall mission strategy.  
	- Triggers the UR5 robotic arm to reset the quadruped’s position once it reaches the end of the path.  

#### 3.4.2 **System Functionality**  

1. The **quadruped** autonomously walks forward along a predefined path, continuously collecting sensor data.  
2. Once it reaches the end of the path, the **UR5 robotic arm** lifts the quadruped and places it back at its starting position, enabling continuous operation.  

By combining **adaptive locomotion** with **robotic manipulation**, this approach ensures an efficient and continuous workflow for autonomous operations.  

### 3.5 Preparation Needs

Successful execution of this project requires a solid understanding of several topics:

- **Robotic Kinematics and Dynamics:** To accurately control the UR5 and quadruped.
- **Sensor Fusion Techniques:** To integrate and filter data from multiple sensor sources.
- **Control Systems and Autonomous Algorithms:** To design robust feedback loops.

Some of these topics will be reviewed in class; however, additional self-study and consultation with experts may be necessary to cover gaps, especially in advanced sensor fusion.

### 3.6 Final Demonstration

Our final demonstration will showcase the complete integration and functioning of the hybrid robotic system:

- **Resources Required:**  
    - UR5 robotic arm and quadruped robot  
    - Sensor suite (IMU, Ultrasonic, Encoders, Camera etc.)  
    - High-performance computing for data processing  
- **Classroom Setup Requirements:**  
	- A designated space for robotic operation with ample room for the quadruped to maneuver  
	- A projection system to display the interface and sensor data  
- **Environmental Variability Handling:**  
	The system will be tested under different lighting and surface textures to ensure that the control algorithms are robust enough to handle variability.
- **Testing & Evaluation Plan:**  
	Multiple test runs will be conducted to verify system performance. We will compare sensor data and simulation outputs against known benchmarks, ensuring that our control strategies adapt correctly to environmental changes.

### 3.7 Impact of the Work

This project has the potential to significantly enhance our understanding of real-time sensor integration and feedback control in hybrid robotic systems. By bridging experimental data with simulation, we expect to drive forward research in autonomous decision-making processes. Additionally, the project will provide valuable insights that can contribute to course development, encouraging further exploration into sensor fusion, control algorithms, and robotic autonomy.

### 3.8 Advising

Our project advisor will be **Dr. Daniel M. Aukes**, who has expressed willingness to provide mentoring and access to specialized hardware resources. Dr. Aukes' guidance will be pivotal in ensuring the project’s technical challenges are addressed effectively. His expectations include regular progress reports, adherence to project milestones, and active participation in troubleshooting sessions. Additional resources, such as access to the lab and advanced simulation tools, have been confirmed to support our experimental setup.

---

## 4. Weekly Milestones (Weeks 7-16)

Below is a table outlining our weekly milestones, covering hardware integration, interface development, sensor handling, and control/autonomy:

| **Week** | **Hardware Integration**                                                  |
|----------|---------------------------------------------------------------------------|
| **7**   | Initialize UR5 for positioning the quadruped at the start point             |
| **8**   | Validate communication between UR5 and host machine                        |
| **9**   | Quadruped calibration and mobility tuning                                  |
| **10**  | Initiate quadruped hardware setup; perform basic mobility tests            |
| **11**  | Fine-tune mechanical alignment and sensor mounts                           |
| **12**  | Full hardware integration test with dynamic movements                      |
| **13**  | Begin integrated system tests (hardware + sensor + control)                |
| **14**  | Final integration and system robustness testing                            |
| **15**  | Optimmizing communication                                                  |
| **16**  | Prepare hardware for final demonstration                                   |

---
*This document outlines our comprehensive plan for the project and serves as the foundation for our website’s main page. Each section details the essential aspects of the project, ensuring clarity in execution and evaluation.*