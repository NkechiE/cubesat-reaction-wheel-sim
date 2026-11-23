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
I_max = 0.16   # drive current limit [A] (will be used by the current limit in the next commit)
f_pwm = 20e3          # switching frequency [Hz]
T_pwm = 1 / f_pwm     # period [s] = 50 µs
duty = 0.4            # duty cycle (0 to 1)

def v_pwm(t):
    # on for the first 'duty' fraction of each period, off for the rest
    return V if (t % T_pwm) < duty * T_pwm else 0.0
def motor_dynamics(t, x):
    i, omega = x
    di_dt = (v_pwm(t)-R*i-Ke*omega)/L   # electrical equation
    domega_dt = (Kt*i-b*omega)/J        # mechanical equation (no load torque)
    return [di_dt, domega_dt]

# 0 to 0.05 s from rest; max_step is well below the 50 µs PWM period so every switching edge is resolved
sol = solve_ivp(motor_dynamics, (0, 0.05), [0, 0], max_step=1e-6)
rpm = sol.y[1] * 60 / (2*np.pi)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(sol.t, sol.y[0])
ax2.plot(sol.t, rpm)
ax1.set_ylabel("Current [A]")
ax1.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [rpm]")
ax2.set_xlabel("Time [s]")
fig.suptitle(f"DC motor with PWM drive ({duty*100:.0f}% duty, {f_pwm/1e3:.0f} kHz)")
# Save two views of the same run to ../figures/
import os
fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
fig.savefig(os.path.join(fig_dir, "step3_pwm_overview.png"), dpi=150)   # full 0 to 0.05 s
ax1.set_xlim(0.0400, 0.0402)   # about 4 switching cycles
fig.savefig(os.path.join(fig_dir, "step3_pwm_zoom.png"), dpi=150)       # zoomed view
plt.show()