% Clear console and variables
clc;
clear all; %#ok<CLALL>
close all force;

% Add path
run ../Lidar/use_lidar
run ../Maps/use_map
run  use_force
addpath('tools_multiagent');


%**************************************************************************
%% Parameters definition
%**************************************************************************
% Print to screen what the script is doing
sec_numb = 1;
fprintf('%d) Loading users params\n',sec_numb);
sec_numb = sec_numb + 1;

mapName = 'Povo2_floor1.txt';

% Define lidar parameter are in generateLIdar function.
fake_lid = generateLidar([],0);
R = fake_lid.RangeMax;
LidarScanArea = defineLidarArea(R);
plot_on = false;
clear R
%
point_closeness = .10; % two lidar points closer that point_closeness are treated as the same oblstacle



%**************************************************************************
%% Load obstacles map and creeate obstacles tree
%**************************************************************************
fprintf('%d) Loading obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;
obstaclesTree = loadPath(mapName);

%**************************************************************************
%% Simulation parameters
%**************************************************************************
dt       = 0.1; % sampling time
time     = 0:dt:45;
n_agents = 1; % number of agents


%**************************************************************************
%% Store simulation data
%**************************************************************************
x                = zeros(length(time),n_agents);
y                = zeros(length(time),n_agents);
theta            = zeros(length(time),n_agents);
v                = zeros(length(time)-1,n_agents);
omega            = zeros(length(time)-1,n_agents);
F_repulsive_vec  = zeros(2, length(time)-1, n_agents);
F_attractive_vec = zeros(2, length(time)-1, n_agents);
wp_index_vec     = zeros(length(time)-1, n_agents);


%% Define initial conditions

fprintf('%d) Define initial conditions of the vehicles \n',sec_numb); sec_numb = sec_numb + 1; 
figure('Name', 'Povo');
hold on;
title('Map','Interpreter','latex');
xlabel('$x_{glob} [m] $','Interpreter','latex');
ylabel('$y_{glob} [m] $','Interpreter','latex');
axis equal;
plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);
for i = 1:n_agents
    display(['insert vehicle position of agent ', num2str(i)]);
    
    
    tmp    = ginput(1);
    x(1,i) = tmp(1);
    y(1,i) = tmp(2);
    
    vehicle_ref_point = plot(tmp(1), tmp(2), '.', 'markersize', 8, 'color', 'k');
    
    display(['insert second point for the heading of vehicle ', num2str(i)]);
    
    tmp2       = ginput(1);
    theta(1,i) = atan2(tmp2(2) - tmp(2), tmp2(1) - tmp(1));
    delete(vehicle_ref_point);
    plot_unicycle(x(1,i), y(1,i), theta(1,i), 'k');
    
end

% pause

%**************************************************************************
%% Plot map
%**************************************************************************
fprintf('%d) Define the waypoints\n',sec_numb); sec_numb = sec_numb + 1; 
for i = 1:n_agents
    plot_unicycle(x(1,i), y(1,i), theta(1,i), 'k');
end
n_waypoints = 4;
waypoints = zeros(2, n_waypoints);
for i = 1:n_waypoints
    display(['insert waypoint ', num2str(i)]);
    tmp = ginput(1);
    waypoints(:,i) = [tmp(1); tmp(2)];
    plot(tmp(1), tmp(2), 'x', 'markersize', 8, 'color', 'r');
end
wp_index = ones(1, n_agents); % each column represents a vehicle. if wp_index(k) = n, it means that vehicle k is approaching the wp number n
display('Type something to continue')
pause
close('Povo');

%% Define potential repulsive functions
% Define repulsive force U_alpha_B (eq.13 SFM)
syms U_0_alpha_B r_alpha_B_x r_alpha_B_y R

r_alpha_B = [r_alpha_B_x; r_alpha_B_y];
U_alpha_B = U_0_alpha_B * exp(-(norm(r_alpha_B))/R);
Grad_U    = gradient(U_alpha_B,[r_alpha_B_x, r_alpha_B_y]); 
% Grad_U    = matlabFunction(Grad_U);
matlabFunction(Grad_U, 'File','Grad_U_repulsive');

clear U_0_alpha_B
clear r_alpha_B_x
clear r_alpha_B_y
clear R
clear Grad_U

%% Define potential attractive functions
% pag 547 Sciavicco
syms k_a e_q_x e_q_y

e_q    = [e_q_x; e_q_y];
U      = k_a * norm(e_q);
Grad_U = gradient(U,[e_q_x, e_q_y]);
matlabFunction(Grad_U, 'File','Grad_U_attractive');

clear k_a
clear e_q_x
clear e_q_y
clear Grad_U

%% Collect all parameters for control in a structure
[R_obstacles, U_0_alpha_B_obstacles] = tune_potential(.3, .4, 3, 0.1);
[R_agents,    U_0_alpha_B_agents]    = tune_potential(.3, .4, 2, 0.01);

ControlParameter.U_0_obstacles                = U_0_alpha_B_obstacles;
ControlParameter.R_obstacles                  = R_obstacles;
ControlParameter.U_0_agents                   = U_0_alpha_B_agents;
ControlParameter.R_agents                     = R_agents;
ControlParameter.k_a                          = 3;
ControlParameter.safe_angle                   = 45*pi/180;
ControlParameter.v_des                        = 1; 
ControlParameter.integral_gain_v_acceleration = 1;
ControlParameter.integral_gain_v_brake        = 3;
ControlParameter.omega_gain                   = 1;
ControlParameter.omega_power                  = 0.5;

ball_wp_radius = .2;
%**************************************************************************
%% Simulation
%**************************************************************************

h_waitbar = waitbar(0, 'Simulating...');
figure('Name', 'Animation');
hold on;

title('Map','Interpreter','latex');
xlabel('$x_{glob} [m] $','Interpreter','latex');
ylabel('$y_{glob} [m] $','Interpreter','latex');
axis equal;
plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);
for i = 1:n_waypoints
    plot(waypoints(1,i), waypoints(2,i), 'x', 'markersize', 8, 'color', 'r');
end
plot_obj = gobjects(n_agents,6); % initialize array of plots
for kk = 1:(length(time)-1)
    for j = 1:n_agents
    %% Simulate LIDAR
    [lidarScan] = simulateLIDAR(LidarScanArea, obstaclesTree, [x(kk,j); y(kk,j); theta(kk,j)]);
    %% Generate angles from lidar scan
    angles = linspace(lidarScan.AngleMin,lidarScan.AngleMax,numel(lidarScan.Ranges));
    
    %% Elaborate lidar data
    % put the obstacles measured in groups
    obst_group              = group_obstacles(angles, lidarScan.Ranges, point_closeness);
    obstacle_points         = point_obst(obst_group, angles, lidarScan.Ranges, [0; 0]); % find the closest point for each obstacle
    for i = 1:size(obstacle_points,2)
        obstacle_points(:,i) = [cos(theta(kk,j)) -sin(theta(kk,j)); sin(theta(kk,j)), cos(theta(kk,j))] * obstacle_points(:,i) + [x(kk,j); y(kk,j)];
    end

    %% Compute the current waypoint
    if norm([x(kk,j); y(kk,j)] - waypoints(:,wp_index(j))) <= ball_wp_radius
        wp_index(j) = wp_index(j) + 1;
    end
    wp_index_vec(kk,j) = wp_index(j);
    %% Compute controls via SFM force
    if kk == 1
        v_old = 0;
    else
        v_old = v(kk-1,j);
    end
    wp_input = waypoints(:, wp_index(j)); % take the wp defined by the planner
    if wp_index(j) == 1
        wp_old = [x(1,j); y(1,j)];
    else
        wp_old = waypoints(:, wp_index(j)- 1 );
    end
    
    
    
    
    
    
    
    plot_obj(j,:) = plot_unicycle(x(kk,j), y(kk,j), theta(kk,j), 'k');
    
    
    
    
    
    
    
    
    
    
    %% Update the waypoint
    if norm([x(kk,j); y(kk,j)] - waypoints(:,wp_index(j))) <= ball_wp_radius
        wp_index(j) = wp_index(j) + 1;
    end
    wp_index_vec(kk,j) = wp_index(j);
    %% Compute rectangle for modified wp
    wp_input = waypoints(:, wp_index(j)); % take the wp defined by the planner
    if wp_index(j) == 1
        wp_old = [x(1,j); y(1,j)];
    else
        wp_old = waypoints(:, wp_index(j)- 1 );
    end
    [wp_input, points_rect] = wp2rectangle(wp_input, wp_old, LidarScanArea, obstaclesTree, x(kk,j), y(kk,j));
    
    if norm([x(kk,j); y(kk,j)] - wp_input)  <= ball_wp_radius
        wp_index(j) = wp_index(j) + 1;
        wp_input = waypoints(:, wp_index(j)); % take the wp defined by the planner
        wp_old = waypoints(:, wp_index(j)- 1 );

        [wp_input, points_rect] = wp2rectangle(wp_input, wp_old, LidarScanArea, obstaclesTree, x(kk,j), y(kk,j));
    end

    
    %% plot
    for jj = 1:4
        plot(points_rect(1,jj), points_rect(2,jj), '.', 'markersize', 8, 'color', 'g');
    end
    plot(wp_input(1), wp_input(2), 'x', 'markersize', 8, 'color', 'c');
    
    
    
    %% 
    [v(kk,j), omega(kk,j), F_repulsive, F_attractive] = SFM_control_multiagents(x(kk,:), y(kk,:), theta(kk,:), j, obstacle_points, ControlParameter, wp_input, v_old, dt);
    F_repulsive_vec(:,kk,j)  = F_repulsive;
    F_attractive_vec(:,kk,j) = F_attractive;
    
    
    
    %% Simulate the dynamics
    % derivatives
    x_dot     = v(kk,j) * cos(theta(kk,j));
    y_dot     = v(kk,j) * sin(theta(kk,j));
    theta_dot = omega(kk,j);
    
    % Euler integration
    x(kk+1,j)     = x(kk,j)     + x_dot     * dt;
    y(kk+1,j)     = y(kk,j)     + y_dot     * dt;
    theta(kk+1,j) = theta(kk,j) + theta_dot * dt;
    end
    waitbar(kk/(length(time)-1));
    delete(plot_obj);
end
close(h_waitbar);


%% Post processing
figure('Name', 'Animation');
hold on;

title('Map','Interpreter','latex');
xlabel('$x_{glob} [m] $','Interpreter','latex');
ylabel('$y_{glob} [m] $','Interpreter','latex');
axis equal;
fig_1_ObstPoly = plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);
% plot waypoints
for i = 1:n_waypoints
    plot(waypoints(1,i), waypoints(2,i), 'x', 'markersize', 8, 'color', 'r');
end

for kk = 1:length(time-1)
    
    
    
    plot_obj = gobjects(n_agents,6); % initialize array of plots
    arrow1 = gobjects(n_agents,1);
    arrow2 = gobjects(n_agents,1);
    for j = 1:n_agents
        
        
        %% Plot forces
        if kk ~= length(time)
            arrow1(j) = quiver(x(kk,j), y(kk,j), F_repulsive_vec(1,kk,j),  F_repulsive_vec(2,kk,j),  'color', 'r', 'linewidth', 2);
            arrow2(j) = quiver(x(kk,j), y(kk,j), F_attractive_vec(1,kk,j), F_attractive_vec(2,kk,j), 'color', 'g', 'linewidth', 2);
        end
        
        %% plot trajectories
        plot(x(1:kk,j), y(1:kk,j), 'linewidth', 1.5, 'color', 'b', 'linestyle',':');
        plot_obj(j,:) = plot_unicycle(x(kk,j), y(kk,j), theta(kk,j), 'k');
    end
    %%
    pause(0.01)
    %%
    delete(arrow1);
    delete(arrow2);
    delete(plot_obj);
end
