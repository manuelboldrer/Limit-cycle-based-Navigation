function [F_repulsive, F_vortex] = react_to_dynamic_obstacle(x_robot, y_robot, x_obstacle, y_obstacle, params, waypoint)
%% Read parameters
U_0_obstacles    = params.U_0_obstacles_react;
R_obstacles      = params.R_obstacles_react;
R_vortex_react   = params.R_vortex_react;
U_0_vortex_react = params.U_0_vortex_react;

%% Relative position
r_alpha_B_x = x_robot - x_obstacle;
r_alpha_B_y = y_robot - y_obstacle;

%% Repulsive force
F_repulsive = - Grad_U_repulsive(R_obstacles, U_0_obstacles, r_alpha_B_x, r_alpha_B_y);

%% Vortex force
F_vortex = Grad_U_repulsive(R_vortex_react, U_0_vortex_react, r_alpha_B_x, r_alpha_B_y);
F_vortex = [F_vortex(2); - F_vortex(1)];
if dot(F_vortex, [waypoint(1) - x_robot , waypoint(2) - y_robot]) <= 0
    F_vortex = -F_vortex;
end