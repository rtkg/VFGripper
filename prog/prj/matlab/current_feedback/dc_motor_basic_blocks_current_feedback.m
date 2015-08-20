%% DC Motor made from basic Simulink blocks
%
% Current feedback control
%
% Model of a DC Motor controlled by PWM signal and H-Bridge.
%
% Input:
%   - desired angular velocity w_d
%   - desired angular acceleration w_d_dot
%   - load torque T_load
% Output:
%   - angular velocity w
%   - angular acceleration w_dot
%   - current i_motor
%
% Orebro University
% Krzysztof Kwiecinski supervised by Robert Krug
%
% 7.08.2015
%
% TODO: calibrate PID

%% Environment
%close all;
clear;


%% Variables
% ----------- DC Motor -------------
L       = 0.746*10^(-3); % [H]
R       = 7.25;          % [ohm]
J       = 8.85*10^(-7);  % [kg*m^2]
k_tau   = 0.0452;        % [Nm/A], torque constant -> torque = I_motor * k_tau
k_emf   = k_tau;         % [V/(rad/s)]
mu      = 10^(-6);       % [N*m/(rad/s)]

%% ----------- PID -----------------
Kp = 1;
Ki = 1;
Kd = 0;

%% Simulation 
t_sim = 0.025;  % [s]
t0    = 0.010;  % [s]
t1    = 0.030;  % [s]

T_load  = 0.1; % [N*m]
w_d     = 500; % [rad/s]
w_d_dot = 0;   % [rad/s^2]

I_d = 0;       % [A]

sim('dc_motor_model_basic_blocks_current_feedback', t_sim);

%% Plotting
figure;
grid on; hold on;
plot(i_motor.time, i_motor.signals.values);
xlabel('time [s]'); ylabel('i motor [A]');
axis([0, t_sim, -5, 5]);
title('i motor(t)');

figure;
grid on; hold on;
plot(w.time, w.signals.values, w.time, w_d);
xlabel('time [s]'); ylabel('w [rad/s]');
%axis([0, t_sim, -1000, 1000]);
title('w(t)');