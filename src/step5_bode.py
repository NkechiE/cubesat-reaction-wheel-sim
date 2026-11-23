import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Motor + wheel parameters (SI units), same as steps 2 to 4
R  = 30        # armature resistance [Ohm]
L  = 1e-3      # armature inductance [H]
Kt = 0.0025    # torque constant [N·m/A]
Ke = 0.0025    # back-EMF constant [V·s/rad]
J  = 1.06e-6   # rotor + flywheel inertia [kg·m²]
b  = 1e-8      # viscous friction [N·m·s/rad]
V  = 5.0       # supply voltage [V]

# Controller gains from step 4 (duty cycle per rad/s of speed error)
Kp = 0.01
Ki = 0.002
cases = [("PI", 0.0, "step5_bode_pi.png"),
         ("PID", 0.005, "step5_bode_pid.png")]

fig_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")

# NOTE: this is the LINEAR model. It ignores the 0 to 1 duty clipping and the
# anti-windup used in step 4, so the margins describe the controller only in
# its unsaturated operating region.

# motor: voltage in -> speed out
G_motor = ct.tf([Kt], [L*J, L*b + R*J, R*b + Kt*Ke])

for name, Kd, filename in cases:
    C = ct.tf([Kd, Kp, Ki], [1, 0])                   # ideal PID: Kd*s + Kp + Ki/s
    L_open = ct.tf(C * V * G_motor, name=f"L_{name}")  # controller + drive gain + motor

    gm, pm, wg, wp = ct.margin(L_open)
    gm_text = "infinite (phase never reaches -180 deg)" if np.isinf(gm) else f"{20*np.log10(gm):.1f} dB at {wg:.1f} rad/s"
    print(f"{name} (Kp = {Kp}, Ki = {Ki}, Kd = {Kd})")
    print(f"  Gain margin:  {gm_text}")
    print(f"  Phase margin: {pm:.1f} deg")
    print(f"  Crossover frequency: {wp:.2f} rad/s ({wp/(2*np.pi):.2f} Hz)")

    plt.figure()
    ct.bode_plot(L_open, dB=True, display_margins=True,
                 title=f"Open-loop Bode plot, {name} (Kp = {Kp}, Ki = {Ki}, Kd = {Kd})")
    plt.savefig(os.path.join(fig_dir, filename), dpi=150)

plt.show()
