%% DC Motor made from basic Simulink blocks
%
% Current feedback control
%
% Model of a DC Motor controlled by PWM signal and H-Bridge.
%
% Input:
%   - desired current I_d
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
close all;
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
Kp = 10;
Ki = 100;
Kd = 0;

%% Simulation 
t_sim = 0.5;  % [s]
t0    = 0.1;  % [s]

T_load  = 0.001; % [N*m]
I_load = T_load/k_emf % [A]
I_d = 0.2;       % [A]
V_MAX = 24;    % [V] 

sim('dc_motor_model_basic_blocks_current_control', t_sim);

%% Plotting
figure;
grid on; hold on;
plot(i_motor.time, i_motor.signals.values, 'b-', i_motor.time, I_d, 'g-');
xlabel('time [s]'); ylabel('i motor [A]');
axis([0, t_sim, -1, 1]);
legend('i motor', 'I_d');
title('i motor(t)');

figure;
grid on; hold on;
plot(pid_out.time, pid_out.signals.values, u_input.time, u_input.signals.values);
xlabel('time [s]'); ylabel('pid output, u _input');
axis([0, t_sim, -1, V_MAX+1]);
legend('pid out, u input');
title('pid output, u input(t)');

figure;
grid on; hold on;
plot(w.time, w.signals.values/44*60/(2*pi())); % 1:44 gear, rpm
xlabel('time [s]'); ylabel('w [rpm]');        
%axis([0, t_sim, -1000, 1000]);
title('w(t)');