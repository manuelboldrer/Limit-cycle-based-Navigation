function [v, omega, F_repulsive, F_attractive] = SFM_control_multiagents_with_local_min_problem(x_vec, y_vec, theta_vec, my_index, obstacle_points, params, waypoint, v_old, dt)
%% Read parameters
U_0_obstacles = params.U_0_obstacles;
k_a           = params.k_a;
R_obstacles   = params.R_obstacles;
angle_cone    = params.safe_angle;
v_des         = params.v_des;
k_i           = params.integral_gain_v_acceleration;
k_brake       = params.integral_gain_v_brake;
k_angular     = params.omega_gain;
alpha         = params.omega_power;
U_0_agents    = params.U_0_agents;
R_agents      = params.R_agents;
d_co_1        = params.d_co_1;
d_co_2        = params.d_co_2;
F_co_max      = params.F_co_max;

x     = x_vec(1,my_index);
y     = y_vec(1,my_index);
theta = theta_vec(1,my_index);
%% Define repulsive force from obstacles
F_repulsive = [0;0];
for i = 1:size(obstacle_points,2)
    r_alpha_B_x = x - obstacle_points(1,i);
    r_alpha_B_y = y - obstacle_points(2,i);
    F_repulsive = F_repulsive - Grad_U_repulsive(R_obstacles, U_0_obstacles, r_alpha_B_x, r_alpha_B_y);
end
%% Define repulsive force from other agents
for i = 1:size(x_vec,2)
    if i ~= my_index
        r_alpha_B_x = x - x_vec(1,i);
        r_alpha_B_y = y - y_vec(1,i);
        F_repulsive = F_repulsive - Grad_U_repulsive(R_agents, U_0_agents, r_alpha_B_x, r_alpha_B_y);
    end
end
%% Define attractive force
% F_attractive = k_a * (waypoint(1) - x)/norm(waypoint(2) - y);
F_attractive = - Grad_U_attractive(x - waypoint(1), y - waypoint(2), k_a);
%% Define cohesion force
% first compute the center of the formation
x_center  = mean(x_vec);
y_center  = mean(y_vec);
me2center = [x_center - x; y_center - y];
% compute force modulus
me2center_norm = norm(me2center);
if me2center_norm <= d_co_1
    force_modulus = 0;
elseif me2center_norm >= d_co_1 && me2center_norm <= d_co_2
    force_modulus = F_co_max/(d_co_2 - d_co_1) * (me2center_norm - d_co_1);
else
    force_modulus = F_co_max;
end
% compute force direction
force_direction = me2center/me2center_norm;
% compute overall force
F_cohesion = force_direction * force_modulus;

%% Compute total force and heading
F_total = F_repulsive + F_attractive + F_cohesion;
h_f     = F_total/norm(F_total); % this is not defined in local minima

%% Compute vehicle heading
h = [cos(theta); sin(theta)];

%% Comnpute v
if dot(h, h_f) >= cos(angle_cone)
    v = v_old - k_i * (v_old - v_des) * dt;
else
    v = v_old - k_brake * v_old * dt;
end

%% Compute omega
omega = -k_angular * (1 - dot(h,h_f))^alpha * sign(h_f(1) * h(2) - h_f(2) * h(1));