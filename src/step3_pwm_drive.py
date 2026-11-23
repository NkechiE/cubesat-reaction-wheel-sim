import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Motor + wheel parameters (SI units), roughly a small CubeSat-class wheel
R  = 30        # armature resistance [Ohm]
L  = 1e-3      # armature inductance [H]
Kt = 0.0025    # torque constant [N·m/A]
Ke = 0.0025    # back-EMF constant [V·s/rad]
J  = 1.06e-6   # rotor + flywheel inertia [kg·m²]
b  = 1e-8      # viscous friction [N·m·s/rad]
V  = 5.0       # applied voltage [V]
I_max = 0.16   # drive current limit [A] (used from step 3)
f_pwm = 20e3          # switching frequency [Hz]
T_pwm = 1 / f_pwm     # period [s] = 50 µs
duty = 0.4            # duty cycle (0 to 1)

def v_pwm(t):
    # on for the first 'duty' fraction of each period, off for the rest
    return V if (t % T_pwm) < duty * T_pwm else 0.0
def motor_dynamics(t, x):
    i, omega = x
    di_dt = (v_pwm(t)-R*i-Ke*omega)/L # TODO 1: di/dt from the electrical equation
    domega_dt = (Kt*i-b*omega)/J # TODO 2: domega/dt from the mechanical equation (assume tau_load = 0)
    return [di_dt, domega_dt]

sol = solve_ivp(motor_dynamics, (0, 0.05), [0, 0], max_step=1e-6) # TODO 3: solve_ivp from t=0 to t=1.5 s, starting at rest (i=0, omega=0)
#         pass max_step=1e-4, because the electrical dynamics are very fast
#max_step= in what time interval to keep solving to get answers to plot gaps
rpm = sol.y[1] * 60 / (2*np.pi)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(sol.t, sol.y[0])
ax2.plot(sol.t, rpm)
ax1.set_ylabel("Current [A]")
ax1.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [rpm]")
ax2.set_xlabel("Time [s]")
fig.suptitle(f"DC motor with PWM drive ({duty*100:.0f}% duty, {f_pwm/1e3:.0f} kHz)")
# Save figure to ../figures/ (added for repo; simulation logic unchanged)
import os
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures", "step3_pwm_drive.png"), dpi=150)
ax1.set_xlim(0.0400, 0.0402)   # about 4 switching cycles
plt.show()
# TODO 4: plot current [A] and speed [rpm] on two subplots
#         rpm = omega * 60 / (2*np.pi)