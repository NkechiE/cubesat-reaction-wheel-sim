% params.m — model parameters for the CubeSat reaction wheel model
% Run this before simulating step6_motor_simulink.slx

% Motor and wheel parameters (SI units)
R  = 30;        % armature resistance [Ohm]
L  = 1e-3;      % armature inductance [H]
Kt = 0.0025;    % torque constant [N*m/A]
Ke = 0.0025;    % back-EMF constant [V*s/rad]
J  = 1.06e-6;   % rotor + flywheel inertia [kg*m^2]
b  = 1e-8;      % viscous friction [N*m*s/rad]
V  = 5.0;       % supply voltage [V]