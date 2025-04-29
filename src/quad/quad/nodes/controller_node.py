import time
import numpy as np
import threading

from std_msgs.msg import Float64MultiArray
from quad.scripts.cpg_controller import CPG

class CPGControllerNode:
    def __init__(self, servo_publisher):
        self.servo_pub = servo_publisher
        self.cpg = CPG()
        self.disabled = True
        self.current_gait = 'pronk'

        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        tp = time.time()
        while True:
            if not self.disabled:
                tc = time.time()
                dt = tc - tp
                tp = tc

                stepped = self.cpg.step(dt)

                msg = Float64MultiArray()
                msg.data = list(stepped)
                self.servo_pub.publish(msg)
            else:
                time.sleep(0.02)
                tp = time.time()
            time.sleep(0.005)

    def set_enable(self, enable):
        self.disabled = not enable

        if not enable:
            reset_msg = Float64MultiArray()
            reset_msg.data = [0.0] * 8
            self.servo_pub.publish(reset_msg)
            print("[CPGController] Disabled: Sent reset + disable servos.")
        else:
            print("[CPGController] Enabled: Starting CPG stepping.")

    def change_gait(self):
        gait_list = list(self.cpg.PHI.keys())
        idx = gait_list.index(self.current_gait)
        idx = (idx + 1) % len(gait_list)
        self.current_gait = gait_list[idx]
        self.cpg = CPG(gait=self.current_gait)

    def update_params(self, params):
        if not self.disabled:
            self.cpg.p = np.array([
                params.get('F', 1.0),
                params.get('DF', 0.5),
                params.get('FO', 0.0),
                params.get('EO', 0.0),
                params.get('HA', 0.0),
                params.get('HO', 0.0),
                params.get('KA', 0.0),
                params.get('KO', 0.0),
            ])
