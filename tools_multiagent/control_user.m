function [v,omega] = control_user(v_old, dt, params)
%% Read parameters
v_des         = params.v_des;
k_i           = params.integral_gain_v_acceleration;

%% Brake
v = v_old - k_i * (v_old - v_des) * dt;
v = max(v,0);
%% Steer the vehicle
omega = 0*pi/180*randn();
