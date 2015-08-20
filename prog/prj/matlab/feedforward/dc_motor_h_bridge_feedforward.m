%% DC Motor with H-Bridge (Simscape library)
%
% Feedforward control
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
% 7.08.2015
%
% TODO:
% NOTES: H-Bridge has limited output voltage

%% Environment
%close all;
clear;

%% Variables
% Voltage supply
VDC = 24; % [V]

%----------- PWM --------------
% duty cycle =  100*(Vref-Vmin)/(Vmax-Vmin) [%],
Vref = VDC;
f_PWM = 2000;  % [Hz]
Vmin  = 0;     % [V]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vmax  = 28;    % [V] , Vmax >= Vref==VDC ---> controls the output voltage
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The output voltage amplitude is set by the Output voltage amplitude.
Vout = 28; % (Vref-Vmin)/(Vmax-Vmin) * 28; % [V] %  V e [5, 28]
Vavg = (Vref-Vmin)/(Vmax-Vmin) * Vout % duty cycle * Vout 

% ----------- H-Bridge ------------
in_en_tresh  = 2.5;  % [V]
in_pwm       = Vavg; % [V]
in_rev_tresh = 2.5;  % [V]
in_br_tresh  = 2.5;  % [V]

h_Vout  = in_pwm; % [V]
h_res   = 0.120;  % [ohm]
h_diode = 0.05;   % [ohm] 

% ----------- DC Motor -------------
k_tau   = 0.0452;         % [Nm/A], torque constant -> torque = I_motor * k_tau
k_emf   = k_tau;          % [V/(rad/s)]
L       = 0.746*10^(-3);  % [H] 
R       = 7.25;           % [ohm]
t_stall = 0.150;          % [N*m]
v_nl    = 5060*2*pi()/60; % [rpm] -> [rad/s]
Un      = 24;             % [V]
I_nl    = 0.0102;         % [A]
mu      = 1e-06;          % [N*m/(rad/s)]
J       = 8.85*10^(-7);   % [kg*m^2]
v0      = 0;              % [rpm]

%% Simulation
t_sim = 0.025;  % [s]
t0    = 0.010;  % [s]
t1    = 0.030;  % [s]

T_load  = 0.1; % [N*m]
w_d     = 400; % [rad/s]
w_d_dot = 0; % [rad/s^2]

U_d = (R*mu/k_tau + k_emf) * w_d + R/k_tau * T_load + R*J/k_tau*w_d_dot
h_Vout = U_d % H-Bridge gives values from [5, 28] V

sim('dc_motor_model_h_bridge_feedforward', t_sim);
 
%% Plotting
figure;
grid on; hold on;
plot(i_motor.time, i_motor.signals.values);
xlabel('time [s]'); ylabel('i motor [A]');
%axis([0, t_sim, -6, 6]);
title('i motor(t)');

figure;
grid on; hold on;
plot(w.time, w.signals.values, 'b', w.time, w_d, 'r*');
xlabel('time [s]'); ylabel('w [rad/s]');
axis( [0, t_sim, 0.95*min(min(w.signals.values), w_d), 1.05*max(max(w.signals.values), w_d) ]);
title('w(t)');