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
I_max = 0.16   # drive current rating [A] (not used yet: this model has no current limit)
omega_target = 10000 * 2*np.pi / 60   # 10,000 rpm in rad/s = about 1047
Kp = 0.01
Ki = 0.002
Kd = 0.005


def motor_dynamics(t, x):
    i, omega, e_int = x
    error = omega_target - omega
    domega_dt = (Kt*i - b*omega) / J
    d_term = -Kd * domega_dt
    duty_raw = Kp*error + Ki*e_int + d_term
    duty = np.clip(duty_raw, 0.0, 1.0)
    v = duty * V
    di_dt = (v - R*i - Ke*omega) / L
    de_int_dt = error if duty_raw == duty else 0.0   # freeze integral when saturated
    return [di_dt, domega_dt, de_int_dt]

sol = solve_ivp(motor_dynamics, (0, 30), [0, 0, 0], method="Radau",
                t_eval=np.linspace(0, 30, 3000))

rpm = sol.y[1] * 60 / (2*np.pi)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(sol.t, sol.y[0])
ax2.plot(sol.t, rpm)
ax1.set_ylabel("Current [A]")
ax1.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [rpm]")
ax2.set_xlabel("Time [s]")
ax2.axhline(10000, linestyle="--", color="gray", label="target")
ax2.legend()
print(f"Final speed: {sol.y[1][-1]*60/(2*np.pi):.0f} rpm, target 10000 rpm")
fig.suptitle(f"PID controller, Kp = {Kp}, Ki = {Ki},  Kd = {Kd}, target = {omega_target*60/(2*np.pi):.0f} rpm")
# Save figure to ../figures/
import os
fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
fig.savefig(os.path.join(fig_dir, "step4_pid_antiwindup.png"), dpi=150)
plt.show()