function [F_total, F_permanent, F_transient] = compute_SFM_forces(neighbors, x, y, theta, obstacle_points, params, waypoint, x_center, y_center, wp_path)
%% Read parameters
U_0_obstacles = params.U_0_obstacles;
k_a           = params.k_a;
R_obstacles   = params.R_obstacles;
U_0_agents    = params.U_0_agents;
R_agents      = params.R_agents;
R_vortex      = params.R_vortex;
U_0_vortex    = params.U_0_vortex;
d_co_1_x      = params.d_co_1_x;
d_co_2_x      = params.d_co_2_x;
F_co_max_x    = params.F_co_max_x;
d_co_1_y      = params.d_co_1_y;
d_co_2_y      = params.d_co_2_y;
F_co_max_y    = params.F_co_max_y;

% x, y, theta are the current state of the robot

%% Define repulsive force from obstacles
F_repulsive_obstacle = [0;0];
for i = 1:size(obstacle_points,2)
    r_alpha_B_x = x - obstacle_points(1,i);
    r_alpha_B_y = y - obstacle_points(2,i);
    F_repulsive_obstacle = F_repulsive_obstacle - Grad_U_repulsive(R_obstacles, U_0_obstacles, r_alpha_B_x, r_alpha_B_y);
end
%% Define vortex field for obstacles
F_vortex = [0;0];
% compute the closest robot of wp_path 
% min_dist  = norm(wp_path(:,1) - [x; y]);
% index_min = 1;
% for i = 2:size(wp_path,2)
%    dist = norm(wp_path(:,i) - [x; y]);
%     if dist < min_dist
%         min_dist  = dist;
%         index_min = i;
%     end
% end
% vortex_point = wp_path(:,index_min);
vortex_point = waypoint;
for i = 1:size(obstacle_points,2)
    r_alpha_B_x = x - obstacle_points(1,i);
    r_alpha_B_y = y - obstacle_points(2,i);
    vortex      = Grad_U_repulsive(R_vortex, U_0_vortex, r_alpha_B_x, r_alpha_B_y);
    vortex      = [vortex(2); - vortex(1)];
    if dot(vortex, [vortex_point(1) - x , vortex_point(2) - y]) <= 0
        vortex = -vortex;
    end
    F_vortex = F_vortex + vortex;
end
%% Define repulsive force from other agents
F_repulsive_agents = [0;0];
for i = 1:size(neighbors,2)
        r_alpha_B_x = x - neighbors(1,i);
        r_alpha_B_y = y - neighbors(2,i);
        F_repulsive_agents = F_repulsive_agents - Grad_U_repulsive(R_agents, U_0_agents, r_alpha_B_x, r_alpha_B_y);
end

%% Define attractive force
% F_attractive = k_a * (waypoint(1) - x)/norm(waypoint(2) - y);
F_attractive = - Grad_U_attractive(x - waypoint(1), y - waypoint(2), k_a);

%% Define cohesion force
me2center = [x_center - x; y_center - y];
%
h          = [cos(theta); sin(theta)]; % vehicle heading
h_perp     = [-sin(theta); cos(theta)]; % orhtogonalto vehicle heading
F_co_x     = cohesion_force(d_co_1_x, d_co_2_x, F_co_max_x, dot(h, me2center));
F_co_y     = cohesion_force(d_co_1_y, d_co_2_y, F_co_max_y, dot(h_perp, me2center));
F_cohesion = F_co_x * h + F_co_y * h_perp;
%% Separate permanent and transient contribution
F_permanent = F_attractive + F_repulsive_obstacle + F_vortex;
F_transient = F_cohesion + F_repulsive_agents;
F_total     = F_permanent + F_transient;
end

function F_co = cohesion_force(d_co_1, d_co_2, F_co_max, scalar_prod)
distance = abs(scalar_prod);
if distance <= d_co_1
    F_co = 0;
elseif distance >= d_co_1 && distance <= d_co_2
    F_co = F_co_max/(d_co_2 - d_co_1) * (distance - d_co_1);
else
    F_co = F_co_max;
end
F_co = F_co * sign(scalar_prod);
end
