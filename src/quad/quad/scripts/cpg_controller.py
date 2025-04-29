from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib as mpl
import time
import threading


class CPGController:
    def __init__(self, bridge):
        self.bridge = bridge

        self.gaits = list(CPG.P0.keys())
        self.gait_index = 0
        self.cpg = CPG(gait=self.gaits[self.gait_index])

        mpl.rcParams['toolbar'] = 'None'
        self.fig = plt.figure('CPG Controller', figsize=(6, 6))
        self.gs = self.fig.add_gridspec(
            nrows=2, ncols=len(self.cpg.P_NAME),
            left=0, right=1, bottom=0.02, top=0.95,
            hspace=0.12, wspace=0,
            height_ratios=[0.92, 0.08]
        )
        self.init_slds()

        self.disabled = True
        self.enable_btn = Button(
            self.fig.add_subplot(self.gs[1, 5:7]),
            'Enable'
        )
        self.enable_btn.on_clicked(self.update_enable_btn)

        self.gait_btn = Button(
            self.fig.add_subplot(self.gs[1, 1:3]),
            self.gaits[self.gait_index].title()
        )
        self.gait_btn.on_clicked(self.update_gait_btn)

        def run():
            while True:
                if not self.disabled:
                    tc = time.time()
                    dt = tc - tp
                    tp = tc

                    self.bridge.set_tx_data(
                        self.bridge.ENABLE_ALL_SREVOS,
                        # self.bridge.DISABLE_ALL_SREVOS,
                        self.cpg.step(dt)
                    )
                else:
                    tp = time.time()
                time.sleep(0.005)  # ~200Hz
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def init_slds(self):
        self.slds = []
        for i in range(len(self.cpg.P_NAME)):
            sld = Slider(
                self.fig.add_subplot(self.gs[0, i]),
                self.cpg.P_NAME[i],
                self.cpg.P_RANGE[i][0], self.cpg.P_RANGE[i][1],
                valinit=self.cpg.p[i], valfmt='%.2f',
                orientation='vertical'
            )
            sld.on_changed(self.update_slds)
            self.slds.append(sld)

    def update_enable_btn(self, event):
        self.disabled = not self.disabled
        if self.disabled:
            self.enable_btn.label.set_text('Enable')
            self.fig.canvas.draw_idle()
            self.bridge.set_tx_data(
                self.bridge.DISABLE_ALL_SREVOS,
                self.bridge.RESET_POS
            )
        else:
            self.enable_btn.label.set_text('Disable')
            self.fig.canvas.draw_idle()
            self.cpg.p = np.array([sld.val for sld in self.slds])

    def update_gait_btn(self, event):
        self.disabled = True
        self.bridge.set_tx_data(
            self.bridge.DISABLE_ALL_SREVOS,
            self.bridge.RESET_POS
        )

        self.gait_index = self.gait_index + 1
        if self.gait_index > len(self.gaits) - 1:
            self.gait_index = 0
        self.gait_btn.label.set_text(self.gaits[self.gait_index].title())
        self.enable_btn.label.set_text('Enable')
        self.fig.canvas.draw_idle()
        self.cpg = CPG(gait=self.gaits[self.gait_index])

        for sld in self.slds:
            sld.ax.remove()
        self.init_slds()

    def update_slds(self, val):
        self.disabled = True
        self.enable_btn.label.set_text('Enable')
        self.bridge.set_tx_data(
            self.bridge.DISABLE_ALL_SREVOS,
            self.bridge.RESET_POS
        )


class CPG:
    # Coupling
    C = np.array([
        [0, 1, 1, 1],
        [1, 0, 1, 1],
        [1, 1, 0, 1],
        [1, 1, 1, 0]
    ])

    # Phase shift
    PHI = {
        'pronk': np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ]),
        'walk': np.array([
            [0, -np.pi / 2, -np.pi, -np.pi / 2 * 3],
            [np.pi / 2, 0, -np.pi / 2, -np.pi],
            [np.pi, np.pi / 2, 0, -np.pi / 2],
            [np.pi / 2 * 3, np.pi, np.pi / 2, 0]
        ]),
        'pace': np.array([
            [0, -np.pi, 0, -np.pi],
            [np.pi, 0, np.pi, np.pi],
            [0, -np.pi, 0, -np.pi],
            [np.pi, 0, np.pi, 0]
        ]),
        'trot': np.array([
            [0, -np.pi, -np.pi, 0],
            [np.pi, 0, 0, np.pi],
            [np.pi, 0, 0, np.pi],
            [0, -np.pi, -np.pi, 0]
        ]),
        'bound': np.array([
            [0, 0, -np.pi, -np.pi],
            [0, 0, -np.pi, -np.pi],
            [np.pi, np.pi, 0, 0],
            [np.pi, np.pi, 0, 0]
        ])

    }

    # Initial parameter
    # frequency, duty factor when swinging forward,
    # flexion/extension offset in fraction of flight
    # hip amplitutde/offset and knee amplitude/offset
    P_NAME = ['F', 'DF', 'FO', 'EO', 'HA', 'HO', 'KA', 'KO']
    P0 = {
        'pronk': np.array([
            4, 0.3, 0, 0,
            0, 0, 1.2, 0,
        ]),
        'walk': np.array([
            1.5, 0.9, 0.3, 0.3,
            0.3, -0.05, 0.8, 0.2,
        ]),
        'pace': np.array([
            3.5, 0.7, 0, 0,
            0.5, 0, 0.5, 0,
        ]),
        'trot': np.array([
            4, 0.5, 0, 0,
            0.6, 0, 0.5, 0,
        ]),
        'bound': np.array([
            4, 0.3, 0.3, 0.3,
            0.5, 0, 0.5, 0,
        ])
    }
    P_RANGE = [
        [0, 5],
        [0, 1],
        [-0.5, 0.5],
        [-0.5, 0.5],
        [-2, 2],
        [-0.5, 0.5],
        [0, 2],
        [-0.5, 0.5]
    ]

    # Initial state
    X0 = {
        'pronk': np.concatenate([np.array([0, 0, 0, 0]), P0['pronk']]),
        'walk': np.concatenate([np.array([0, np.pi / 2, np.pi, np.pi / 2 * 3]), P0['walk']]),
        'pace': np.concatenate([np.array([0, np.pi, 0, np.pi]), P0['pace']]),
        'trot': np.concatenate([np.array([0, np.pi, np.pi, 0]), P0['trot']]),
        'bound': np.concatenate([np.array([0, 0, np.pi, np.pi]), P0['bound']])
    }

    # Converge rate
    ALPHA_PHI = 1
    ALPHA_P = 5

    def __init__(self, gait='pronk'):
        self.gait = gait

        # Phase shift
        self.phi = self.PHI[self.gait]

        # Oscillator and parameter state
        self.x = CPG.X0[self.gait]

        # Parameters
        self.p = CPG.P0[self.gait]

        # Control input
        # hip_fl, hip_fr, hip_rl, hip_rr, knee_fl, knee_fr, knee_rl, knee_rr
        self.u = np.zeros(8)

        self.servo_offset = np.array([
            3.04, 3.27, 3.27, 3.04, 1.03, 5.20, 5.20, 1.03
        ])
        self.servo_direction = np.array([
            -1, 1, -1, 1, 1, -1, -1, 1
        ])
        self.u_to_servo = np.array([
            0, 1, 2, 3, 4, 5, 6, 7
        ])

    def dx_fn(self, t, x):
        dx = np.zeros(len(x))
        phis = x[:4].reshape((-1, 1))

        # Parameters
        dx[4:] = CPG.ALPHA_P * (self.p - x[4:])

        # Oscillator phase
        for i in range(4):
            phii = phis[i]

            coeff1 = np.eye(4)
            coeff1[i, i] = 0
            coeff2 = np.ones((4, 1))
            coeff2[i, 0] = 0
            sigma = np.sum(self.ALPHA_PHI * self.C[[i], :].T * np.sin(
                coeff1 @ phis - coeff2 * phii - coeff2 * self.phi[[i], :].T
            ))

            f = x[4]
            dphii = 2 * np.pi * f + sigma
            dx[i] = dphii

        return dx

    def u_fn(self, t, x):
        u = np.zeros(8)
        f, df, fo, eo = x[4:8]
        for i in range(4):
            phii = np.fmod(x[i], 2 * np.pi)
            amphi, offhi, ampki, offki = x[8:8 + 4]

            # Hip
            if phii <= 2 * np.pi * df:
                thetai = phii / (2 * np.pi * df) * np.pi
            else:
                thetai = (
                    (phii - 2 * np.pi * df) /
                    (2 * np.pi * (1 - df)) * np.pi
                ) + np.pi
            hi = amphi * np.cos(thetai) + offhi
            u[i] = hi

            # Knee
            fs = np.fmod(df + fo * (1 - df), 1)
            es = np.fmod(1 + eo * (1 - df), 1)
            if fs < es and phii > 2 * np.pi * fs and phii < 2 * np.pi * es:
                si = 1
            elif fs >= es and (phii < 2 * np.pi * es or phii > 2 * np.pi * fs):
                si = 1
            else:
                si = 0
            ki = si * ampki + offki
            u[i + 4] = ki

        return u

    def step(self, dt):
        self.x = self.x + self.dx_fn(0, self.x) * dt
        self.u = self.u_fn(0, self.x).flatten()

        return (
            self.u[self.u_to_servo] * self.servo_direction +
            self.servo_offset
        )


def solve_ivp2(dx, tf, x0, step=1e-3):
    num_step = int(tf / step)
    t = np.arange(num_step) * step
    y = np.zeros((len(x0), num_step))

    x = x0
    for i, tc in enumerate(t):
        x = x + dx(tc, x) * step
        y[:, i] = x

    class sol:
        pass
    sol.t = t
    sol.y = y

    return sol


if __name__ == "__main__":
    c = CPG()

    tf = 5
    # sol = solve_ivp(c.dx_fn, [0, tf], c.x, max_step=1e-3)
    sol = solve_ivp2(c.dx_fn, tf, c.x, step=1e-3)

    labels = ['FL', 'FR', 'RL', 'RR']
    styles = ['-', '-', '--', '--']
    colors = ['C0', 'C1', 'C2', 'C3']

    plt.figure('Oscillator')
    for i in range(4):
        sol.y[i, :] = np.fmod(sol.y[i, :], 2 * np.pi)
        plt.plot(
            sol.t, sol.y[i, :],
            styles[i], color=colors[i], label=labels[i]
        )
    plt.ylabel('Phase (rad)')
    plt.legend()

    us = np.zeros((8, len(sol.t)))
    for j, (t, x) in enumerate(zip(sol.t, sol.y.T)):
        us[:, j] = c.u_fn(t, x)
    us = np.array(us)

    plt.figure('Output')
    for i in range(4):
        plt.subplot(211)
        plt.plot(sol.t, us[i, :], styles[i], color=colors[i], label=labels[i])
        plt.title('Hip')

        plt.subplot(212)
        plt.plot(
            sol.t, us[i + 4, :], styles[i], color=colors[i], label=labels[i]
        )
        plt.title('Knee')

    plt.show()
