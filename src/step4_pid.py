import os
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
target_rpm = omega_target * 60 / (2*np.pi)

# Controller cases: (name, Kp, Ki, Kd, anti_windup, figure file)
cases = [
    ("P only, Kp=0.001",     0.001, 0.0,   0.0,   False, "step4_p_kp001.png"),
    ("P only, Kp=0.01",      0.01,  0.0,   0.0,   False, "step4_p_kp01.png"),
    ("PI, no anti-windup",   0.01,  0.002, 0.0,   False, "step4_pi_windup.png"),
    ("PI with anti-windup",  0.01,  0.002, 0.0,   True,  "step4_pi_antiwindup.png"),
    ("PID with anti-windup", 0.01,  0.002, 0.005, True,  "step4_pid_antiwindup.png"),
]

fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")


def make_dynamics(Kp, Ki, Kd, anti_windup):
    def motor_dynamics(t, x):
        i, omega, e_int = x
        error = omega_target - omega
        domega_dt = (Kt*i - b*omega) / J
        d_term = -Kd * domega_dt
        duty_raw = Kp*error + Ki*e_int + d_term
        duty = np.clip(duty_raw, 0.0, 1.0)
        v = duty * V
        di_dt = (v - R*i - Ke*omega) / L
        if anti_windup:
            de_int_dt = error if duty_raw == duty else 0.0   # freeze integral when saturated
        else:
            de_int_dt = error                                # always integrate
        return [di_dt, domega_dt, de_int_dt]
    return motor_dynamics


def settling_time(t, rpm, band=0.02):
    # time after which speed stays within +/- band of the target; None if it never does
    outside = np.abs(rpm - target_rpm) > band * target_rpm
    if not outside.any():
        return t[0]
    last = np.where(outside)[0][-1]
    return t[last + 1] if last + 1 < len(t) else None


for name, Kp, Ki, Kd, anti_windup, filename in cases:
    sol = solve_ivp(make_dynamics(Kp, Ki, Kd, anti_windup), (0, 30), [0, 0, 0],
                    method="Radau", t_eval=np.linspace(0, 30, 3000))
    rpm = sol.y[1] * 60 / (2*np.pi)

    peak = rpm.max()
    overshoot = max(0.0, (peak - target_rpm) / target_rpm * 100)
    ts = settling_time(sol.t, rpm)
    ts_text = f"{ts:.2f} s" if ts is not None else "not settled within 30 s"
    print(f"{name}: final {rpm[-1]:.1f} rpm, peak {peak:.1f} rpm, "
          f"overshoot {overshoot:.2f} %, 2% settling time {ts_text}")

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.plot(sol.t, sol.y[0])
    ax2.plot(sol.t, rpm)
    ax1.set_ylabel("Current [A]")
    ax1.set_xlabel("Time [s]")
    ax2.set_ylabel("Speed [rpm]")
    ax2.set_xlabel("Time [s]")
    ax2.axhline(target_rpm, linestyle="--", color="gray", label="target")
    ax2.legend()
    fig.suptitle(f"{name}\nKp = {Kp}, Ki = {Ki}, Kd = {Kd}, target = {target_rpm:.0f} rpm")
    fig.savefig(os.path.join(fig_dir, filename), dpi=150)

plt.show()
