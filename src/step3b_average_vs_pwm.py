import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Motor + wheel parameters (SI units), same as step3_pwm_drive.py
R  = 30        # armature resistance [Ohm]
L  = 1e-3      # armature inductance [H]
Kt = 0.0025    # torque constant [N·m/A]
Ke = 0.0025    # back-EMF constant [V·s/rad]
J  = 1.06e-6   # rotor + flywheel inertia [kg·m²]
b  = 1e-8      # viscous friction [N·m·s/rad]
V  = 5.0       # supply voltage [V]
f_pwm = 20e3          # switching frequency [Hz]
T_pwm = 1 / f_pwm     # period [s] = 50 µs
duty = 0.4            # duty cycle (0 to 1)
t_end = 0.05          # simulated time [s]

def v_pwm(t):
    # on for the first 'duty' fraction of each period, off for the rest
    return V if (t % T_pwm) < duty * T_pwm else 0.0

def v_avg(t):
    # averaged model: constant voltage equal to the PWM mean
    return duty * V

def make_dynamics(v_func):
    def motor_dynamics(t, x):
        i, omega = x
        di_dt = (v_func(t) - R*i - Ke*omega) / L   # electrical equation
        domega_dt = (Kt*i - b*omega) / J           # mechanical equation (no load torque)
        return [di_dt, domega_dt]
    return motor_dynamics

# Same solver settings for both runs so the comparison is fair
sol_pwm = solve_ivp(make_dynamics(v_pwm), (0, t_end), [0, 0], max_step=1e-6)
sol_avg = solve_ivp(make_dynamics(v_avg), (0, t_end), [0, 0], max_step=1e-6)

rpm_pwm = sol_pwm.y[1] * 60 / (2*np.pi)
rpm_avg = sol_avg.y[1] * 60 / (2*np.pi)

end_pwm = rpm_pwm[-1]
end_avg = rpm_avg[-1]
diff_pct = (end_pwm - end_avg) / end_avg * 100
print(f"Speed at t = {t_end} s, PWM model:      {end_pwm:.3f} rpm")
print(f"Speed at t = {t_end} s, averaged model: {end_avg:.3f} rpm")
print(f"Difference (PWM vs averaged):         {diff_pct:+.3f} %")

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(sol_pwm.t, rpm_pwm, label="PWM")
ax1.plot(sol_avg.t, rpm_avg, "--", label="Averaged (duty × V)")
ax1.set_ylabel("Speed [rpm]")
ax1.legend()
ax2.plot(sol_pwm.t, sol_pwm.y[0], label="PWM")
ax2.plot(sol_avg.t, sol_avg.y[0], "--", label="Averaged (duty × V)")
ax2.set_ylabel("Current [A]")
ax2.set_xlabel("Time [s]")
ax2.legend()
fig.suptitle(f"Averaged model vs PWM ({duty*100:.0f}% duty, {f_pwm/1e3:.0f} kHz)")

fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
fig.savefig(os.path.join(fig_dir, "step3b_average_vs_pwm.png"), dpi=150)
plt.show()
