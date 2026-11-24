import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

R, L, Kt, Ke = 30, 1e-3, 0.0025, 0.0025
J, b, V = 1.06e-6, 1e-8, 5.0

def motor_dynamics(t, x):
    i, omega = x
    return [(V - R*i - Ke*omega)/L, (Kt*i - b*omega)/J]

# Python run
sol = solve_ivp(motor_dynamics, (0, 30), [0, 0], method="Radau",
                t_eval=np.linspace(0, 30, 3000))
t_py, w_py = sol.t, sol.y[1]

# Simulink data
here = os.path.dirname(os.path.abspath(__file__))
data = np.loadtxt(os.path.join(here, "..", "simulink", "simulink_speed.csv"),
                  delimiter=",")
t_sl, w_sl = data[:, 0], data[:, 1]

# compare on the Simulink time points
w_py_interp = np.interp(t_sl, t_py, w_py)
diff = np.abs(w_py_interp - w_sl)
print(f"Max difference: {diff.max():.2f} rad/s "
      f"({100*diff.max()/w_py.max():.3f}% of final speed)")

plt.figure(figsize=(8, 5))
plt.plot(t_py, w_py, label="Python (solve_ivp)", linewidth=2)
plt.plot(t_sl, w_sl, "--", label="Simulink", linewidth=2)
plt.xlabel("Time [s]"); plt.ylabel("Speed [rad/s]")
plt.title("Open-loop spin-up: Python vs Simulink")
plt.legend(); plt.grid(True)
plt.savefig(os.path.join(here, "..", "figures", "step7_comparison.png"), dpi=150)
plt.show()