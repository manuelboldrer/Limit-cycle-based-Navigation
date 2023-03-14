function F_rep = dynamic_obstacle_velocity_force(x, y, theta, v, x_obstacle, y_obstacle, v_obs, params)
%% parameters
a_max = .5; % max robot deceleration along the line from robot to obstacle
eta   = 0.7; % gain for the forces
rho_0 = 1.5;
%% Use paper notation
v     = v*[cos(theta); sin(theta)]; % vectorial velocity of the robot
% v_obs = vectorial velocity of the robot
n_RO  = -[x - x_obstacle; y - y_obstacle]/norm([x - x_obstacle; y - y_obstacle]); % this unit vector points from robot to obstacle
v_RO  = dot(n_RO,v-v_obs); % relative velocity from robot to obstacle; negative if the robot is moving away
rho_s = norm([x - x_obstacle; y - y_obstacle]); % distance between robot and obstacle
rho_m = v_RO^2/(2*a_max);
%% Compute repulsive force along the line from robot to obstacle 
F_rep_1 = -eta * (rho_s - rho_m^2) * (1 + v_RO/a_max) * n_RO;
%% Compute repulsive force along the direction orthogonal to the line from robot to obstacle 
n_RO_perp = [-n_RO(2); n_RO(1)];
% v_RO_perp = sqrt(norm(v - v_obs) - v_RO^2);
v_RO_perp = dot(n_RO_perp,v-v_obs);
F_rep_2   = eta * v_RO * v_RO_perp / (rho_s * a_max * (rho_s - rho_m)^2 )* n_RO_perp;
if rho_s - rho_m >= rho_0 || v_RO <=0
    F_rep = [0;0];
    1
elseif rho_s - rho_m > 0 && rho_s - rho_m < rho_0 && v_RO > 0
    F_rep = F_rep_1 + F_rep_2;
    2
else
    F_rep = [0;0]+ F_rep_1 + F_rep_2;
    3
end
%   F_rep = F_rep_1 + F_rep_2;
if norm(F_rep) > params.k_a
%     F_rep = F_rep/norm(F_rep)*params.k_a;
end
