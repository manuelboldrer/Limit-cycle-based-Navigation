function [v,omega] = control_robot(theta, h_reference, v_old, dt, params, h_prec)
%% Read parameters
k_brake       = params.integral_gain_v_brake;
k_angular     = params.omega_gain;
alpha         = params.omega_power;
%% Compute vehicle heading
h = [cos(theta); sin(theta)]; 
%% Brake
v = v_old - k_brake * v_old * dt;
v = max(v,0);
%% Steer the vehicle
omega = -k_angular * (1 - dot(h, h_reference))^alpha * sign(h_reference(1) * h(2) - h_reference(2) * h(1));

% thetaD_dot = (atan2(h_reference(2),h_reference(1)) - atan2(h_prec(2),h_prec(1)))/dt;
% omega = thetaD_dot + 0.01*(theta - atan2(h_reference(2),h_reference(1)));
