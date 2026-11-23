# CubeSat Reaction Wheel: Closed-Loop Drive Simulation

## Project goal

This project simulates the drive of a small CubeSat-class reaction wheel: a brushed DC motor model, a PWM motor driver, and a PID speed controller. The stability of the closed loop will then be checked with frequency-domain tools. The aim is to build the full chain step by step in Python, then rebuild it in Simulink and compare the two.

## Roadmap

- [x] Step 2: Open-loop DC motor model, scaled to a CubeSat wheel
- [x] Step 3: PWM drive
- [x] Step 4: PID speed control
- [x] Step 5: Stability analysis (Bode plot, gain and phase margin)
- [ ] Step 6: Simulink rebuild
- [ ] Step 7: Python vs Simulink comparison

## Model parameters

| Parameter | Value | Origin |
|---|---|---|
| Supply voltage V | 5 V | From public product listing, to be verified |
| Armature resistance R | 30 Ω | My modelling choice |
| Armature inductance L | 1 mH | My modelling choice |
| Back-EMF / torque constant Ke = Kt | 0.0025 V·s/rad (= N·m/A) | My modelling choice |
| Rotor + wheel inertia J | 1.06e-6 kg·m² | Derived from listed momentum ÷ listed max speed (2 mNms ÷ 18,000 rpm), to be verified |
| Viscous friction b | 1e-8 N·m·s/rad | My modelling choice |
| Drive current limit I_max | 0.16 A (not used until Step 3) | My modelling choice |
| Peak torque (target) | 0.4 mNm | From public product listing, to be verified |
| Max speed (target) | 18,000 rpm | From public product listing, to be verified |
| Angular momentum (target) | 2 mNms | From public product listing, to be verified |

None of these values are taken from a manufacturer datasheet.

## Results

### Step 2: Open-loop spin-up

![Step 2 spin-up](figures/step2_spinup.png)

This shows a 5 V step applied to the motor at rest, simulated for 30 s:

- **Current** (top) jumps almost immediately to its peak of about 0.166 A. It then decays smoothly as back-EMF builds up, ending at about 0.008 A at t = 30 s.
- **Speed** (bottom) rises like a first-order system. It reaches 63% of its 30 s value after about 4.8 s, 95% after about 14.4 s, and is nearly flat at about 18,190 rpm by t = 30 s.

For comparison with the listed targets: the simulated peak torque (Kt × peak current) is about 0.42 mNm, and the momentum at t = 30 s (J × ω) is about 2.02 mNms. The peak current (0.166 A) goes slightly above I_max (0.16 A), because this open-loop model has no current limit yet. Current limiting is planned for Step 3.

### Step 3: PWM drive

PWM (pulse-width modulation) switches the full supply voltage on and off very quickly. Changing the duty cycle changes the *average* voltage the motor sees. The switch is either fully on or fully off, so little power is lost as heat, unlike a linear driver that burns the extra voltage. Simulating every 50 µs switching edge needs microsecond time steps, so PWM is only simulated in short windows. Long runs (PID, stability) will use the averaged model, after checking below that it matches.

Settings: 5 V supply, 20 kHz switching (50 µs period), 40% duty cycle, starting at rest, 0 to 0.05 s.

**Zoom (t = 40.0 to 40.2 ms, about 4 switching cycles)**

![Step 3 PWM zoom](figures/step3_pwm_zoom.png)

The current follows a saw-tooth. It rises during the on-time and falls during the off-time, between about 0.039 A and 0.096 A. Over this short window the speed is flat at about 60 rpm.

**Overview (0 to 0.05 s)**

![Step 3 PWM overview](figures/step3_pwm_overview.png)

At this scale the ripple looks like a solid band. After a very short start-up, the current settles into the same 0.04 to 0.10 A band. The speed rises almost linearly and reaches about 74.7 rpm at 0.05 s. This is the very start of a spin-up that takes seconds (see Step 2).

**Averaged model vs PWM (0 to 0.05 s, same solver settings)**

![Step 3b averaged model vs PWM](figures/step3b_average_vs_pwm.png)

The averaged model replaces the switching voltage with a constant duty × V = 2 V. The two speed curves lie on top of each other. At 0.05 s, the PWM run reaches 74.659 rpm and the averaged run 74.639 rpm, a difference of +0.026%. The averaged current is a flat line at about 0.066 A, which matches the mean of the PWM ripple over the last 10 ms (0.0661 A). The averaged model therefore loses the ripple but keeps the speed and mean current, which is what the speed controller cares about.

### Step 3c: Current limit (demo)

The drive switches the voltage off whenever the current reaches a limit, and back on once it drops below it. The demo uses 100% duty, so without a limit the current would head for the full stall value. The limit is set to 0.08 A on purpose, half of I_max (0.16 A), so its effect is easy to see. The run covers 0 to 0.01 s with `max_step=1e-7`.

**Zoom (first 500 µs)**

![Step 3c current limit zoom](figures/step3c_current_limit_zoom.png)

**Overview (0 to 0.01 s)**

![Step 3c current limit overview](figures/step3c_current_limit_overview.png)

The current rises from zero and reaches 0.08 A after about 22 µs. From then on it stays pinned at the limit (0.07999 to 0.08003 A after 5 ms). With the current held constant, the torque is constant too, so the speed rises in a straight line to about 18 rpm at 0.01 s.

Note: this limiter switches on and off at every solver step around the limit, so in the model its switching rate is set by `max_step`, not by real hardware. A real driver would usually limit once per PWM cycle or use a small hysteresis band.

### Step 4: PID speed control

A PID controller sets the duty cycle (clipped to 0 to 1) from the speed error. The motor sees the averaged voltage duty × V, as checked in Step 3b. The derivative term acts on the measured acceleration rather than the error. The integral is frozen while the duty cycle is saturated (anti-windup). Gains: Kp = 0.01, Ki = 0.002, Kd = 0.005. Target: 10,000 rpm from rest, simulated for 30 s.

![Step 4 PID with anti-windup](figures/step4_pid_antiwindup.png)

- **Speed** rises with the duty cycle stuck at 100% until about 2.8 s, reaches 90% of the target at about 3.5 s, and stays within 2% of the target after about 5.6 s. It approaches the target from below without overshooting and ends at 9,999.5 rpm at 30 s.
- **Current** starts at about 0.166 A (the same start-up peak as Step 2, slightly above I_max, because there is no current limit in this model). It drops sharply once the controller leaves saturation, then settles at about 0.004 A, with a final duty cycle of about 0.55.

### Step 5: Stability analysis (Bode plots)

The open loop (controller × supply voltage × motor) is analysed for the PI controller (Kd = 0) and the PID controller (Kd = 0.005), with Kp = 0.01 and Ki = 0.002 in both. This is the linear model: it ignores the duty clipping and anti-windup, so the margins only describe the controller in its unsaturated operating region.

- **Gain margin:** how much the loop gain could be multiplied before the system becomes unstable.
- **Phase margin:** how much extra delay (phase lag) the loop could tolerate at the crossover frequency before it becomes unstable.

![Step 5 Bode plot, PI](figures/step5_bode_pi.png)

![Step 5 Bode plot, PID](figures/step5_bode_pid.png)

| Controller | Gain margin | Phase margin | Crossover frequency |
|---|---|---|---|
| PI (Kd = 0) | Infinite | 90.1° | 3.93 rad/s (0.63 Hz) |
| PID (Kd = 0.005) | Infinite | 120.6° | 50,760 rad/s (8,079 Hz) |

**Conclusion.** The gain margin is infinite in both cases because the phase never reaches −180°, and both phase margins are healthy. However, the PID crossover is at about 8 kHz, in the electrical frequency range. A real digital controller would need a sample rate far above that, and the averaged PWM model from Step 3b is not valid that close to the 20 kHz switching frequency. The PI crossover is at about 4 rad/s, in the mechanical range, and is easy to implement. The D term is therefore not earning its place: in Step 4 it was the anti-windup, not the D term, that removed the overshoot.

## How to run

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate    macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python src/step2_motor_model.py
python src/step3_pwm_drive.py
python src/step3b_average_vs_pwm.py
python src/step3c_current_limit.py
python src/step4_pid.py
python src/step5_bode.py
```

Each script opens a plot window and saves its figures to `figures/`.

## Repository layout

```
README.md
requirements.txt
src/
  step2_motor_model.py    # Step 2: open-loop motor model
  step3_pwm_drive.py      # Step 3: PWM drive
  step3b_average_vs_pwm.py  # Step 3b: averaged model vs PWM
  step3c_current_limit.py   # Step 3c: current limit demo
  step4_pid.py              # Step 4: PID speed control with anti-windup
  step5_bode.py             # Step 5: open-loop Bode plots and stability margins (PI and PID)
figures/                  # plots saved by the scripts
simulink/                 # Step 6 models (placeholder)
```
