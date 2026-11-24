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

def motor_dynamics(t, x):
    i, omega = x
    di_dt = (V-R*i-Ke*omega)/L # TODO 1: di/dt from the electrical equation
    domega_dt = (Kt*i-b*omega)/J # TODO 2: domega/dt from the mechanical equation (assume tau_load = 0)
    return [di_dt, domega_dt]

sol = solve_ivp(motor_dynamics, (0, 30), [0, 0], method="Radau",
                t_eval=np.linspace(0, 30, 3000)) # TODO 3: solve_ivp from t=0 to t=1.5 s, starting at rest (i=0, omega=0)
#         pass max_step=1e-4, because the electrical dynamics are very fast
#max_step= in what time interval to keep solving to get answers to plot gaps
rpm = sol.y[1] * 60 / (2*np.pi)

# Numbers quoted in the README
i_sim = sol.y[0]
print(f"Peak current: {i_sim.max():.4f} A, current at t = 30 s: {i_sim[-1]:.4f} A")
print(f"Speed at t = 30 s: {rpm[-1]:.1f} rpm")
print(f"Time to 63% of the 30 s speed: {sol.t[np.argmax(rpm >= 0.632*rpm[-1])]:.2f} s")
print(f"Time to 95% of the 30 s speed: {sol.t[np.argmax(rpm >= 0.95*rpm[-1])]:.2f} s")
print(f"Peak torque (Kt x peak current): {Kt*i_sim.max()*1e3:.3f} mNm")
print(f"Momentum at t = 30 s (J x omega): {J*sol.y[1][-1]*1e3:.3f} mNms")

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(sol.t, sol.y[0])
ax2.plot(sol.t, rpm)
ax1.set_ylabel("Current [A]")
ax1.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [rpm]")
ax2.set_xlabel("Time [s]")
fig.suptitle("DC motor step response (5 V applied at t = 0)")
# Save figure to ../figures/ (added for repo; simulation logic unchanged)
import os
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures", "step2_spinup.png"), dpi=150)
plt.show()
# TODO 4: plot current [A] and speed [rpm] on two subplots
#         rpm = omega * 60 / (2*np.pi)