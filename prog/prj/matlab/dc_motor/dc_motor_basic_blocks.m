close all;
clear;

% ----------- DC Motor -------------
L       = 0.746*10^(-3); % [H] 
R       = 7.25;          % [ohm]
J       = 8.85*10^(-7);  % [kg*m^2]
k_tau   = 0.0452;        % [Nm/A], torque constant -> torque = I_motor * k_tau
k_emf   = k_tau;         % [V/(rad/s)]
mu      = 10^(-6);       % [N*m/(rad/s)]


%% Simulation 
t_sim = 0.1; % [s]
u      = 24;  % [V]
T_load = -0.0; % [N*m]


sim('dc_motor_model_basic_blocks', t_sim);

%% Plotting
figure;
grid on; hold on;
plot(i_motor.time, i_motor.signals.values);
xlabel('time [s]'); ylabel('i motor [A]');
%axis([0, t_sim, -6, 6]);
title('i motor(t)');

figure;
grid on; hold on;
plot(w.time, w.signals.values);
xlabel('time [s]'); ylabel('w [rad/s]');
%axis([0, t_sim, -10000, 10000]);
title('w(t)');

figure;
grid on; hold on;
plot(w.time, w.signals.values*k_emf);
xlabel('time [s]'); ylabel('E_b [V]');
%axis([0, t_sim, -10000, 10000]);
title('E_b(t)');