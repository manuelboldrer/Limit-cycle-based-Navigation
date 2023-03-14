%% Clear console and variables
clc;
clear all; %#ok<CLALL>
close all force;
restoredefaultpath;
%% Add path
addpath('tools_multiagent');
addpath('Utility');

%% *****ETH Flag
ETH_flag = 0;
%**************************************************************************
%% Simulation parameters
%**************************************************************************surf([0:0.19:1.9],[0:0.15:1.5],curv)
xlabel('$x_{trasl}$','interpreter','latex')
ylabel('$\nu$','interpreter','latex')
dt                = 0.1; % sampling time
t_end             = 40;
time              = 0:dt:t_end;
n_obstacles       = 1;
n_agents          = 1; % number of agents
n_waypoints       = 3; % number of waypoints
%% Parameters for egg shape
F_max       =  5   ;
egg.a       =  2   ;
egg.epsilon =  0.5 ;
egg.b       =  egg.epsilon * egg.a ;
egg.alpha   =  0.5   ;
egg.trasl   =  egg.a*0.75 ;
%% *****Flags waypoints and initial conditions
manual_wp    = 0 ;
manual_ics   = 0 ;
%% Choose the way to assign wp
wp_method = 4 ;
% 0 = pass the wp
% 1 = use restangle
% 2 = create a path of wp
% 3 = use the lidar simulation to select the path of wp
index_prev_vec = ones(n_agents,1);
%% Choose how to estimate the center
center_algo_flag = 2;
% if 1, basic mean
% if 2, mewtropolis mean
%% Video flag
video_flag = 0;
%**************************************************************************
%% Store simulation data
%**************************************************************************
x                      = zeros(length(time),n_agents);
y                      = zeros(length(time),n_agents);
theta                  = zeros(length(time),n_agents);
v                      = zeros(length(time)-1,n_agents);
omega                  = zeros(length(time)-1,n_agents);
hysteresis_flag        = zeros(length(time),n_agents);
h_reference_vec        = zeros(2, length(time)-1, n_agents);
F_total_vec            = zeros(2, length(time)-1, n_agents);
F_permanent_vec        = zeros(2, length(time)-1, n_agents);
wp_index_vec           = zeros(length(time)-1, n_agents);
wp_input_vec           = zeros(2,length(time));
center_matrix_store    = zeros(n_agents,2,length(time));
x_obstacle             = zeros(length(time),n_obstacles);
y_obstacle             = zeros(length(time),n_obstacles);
x_obstacle_equivalent  = zeros(length(time),n_obstacles);
y_obstacle_equivalent  = zeros(length(time),n_obstacles);
vx_obstacle_equivalent = zeros(length(time),n_obstacles);
vy_obstacle_equivalent = zeros(length(time),n_obstacles);
rot_matrix_store       = zeros(n_agents,n_obstacles,2,2);
obstacle_points        = [0;0];
count                  = zeros(n_obstacles,n_agents);
x_obstacle(1,1:n_obstacles)   =  [4.0];
y_obstacle(1,1:n_obstacles)   =  [5.2];
direction1                    =  zeros(n_obstacles,n_agents);
direction_pre                 =  zeros(n_obstacles,n_agents);
%% Define initial conditions
if manual_ics == 1
    hold on;
    title('Map','Interpreter','latex');
    grid on;
    xlabel('$x_{glob} [m] $','Interpreter','latex');
    ylabel('$y_{glob} [m] $','Interpreter','latex');
    axis equal;
    axis([0 10 0 10])
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
    x_1     = x(1,:);
    y_1     = y(1,:);
    theta_1 = theta(1,:);
    save('x_1','x_1');
    save('y_1','y_1');
    save('theta_1','theta_1');
else
    load('x_1');
    load('y_1');
    load('theta_1');
    
    x(1,:)     = x_1;
    y(1,:)     = y_1;
    theta(1,:) = theta_1;
    
end
%% Define waypoints

if manual_wp == 1
    for i = 1:n_agents
        plot_unicycle(x(1,i), y(1,i), theta(1,i), 'k');
    end
    waypoints = zeros(2, n_waypoints);
    for i = 1:n_waypoints
        display(['insert waypoint ', num2str(i)]);
        tmp = ginput(1);
        waypoints(:,i) = [tmp(1); tmp(2)];
        plot(tmp(1), tmp(2), 'x', 'markersize', 8, 'color', 'r');
    end
    save('waypoints.mat')
else
    load('waypoints.mat');
end
if manual_ics == 1
    wp_index = ones(1, n_agents); % each column represents a vehicle. if wp_index(k) = n, it means that vehicle k is approaching the wp number n
    close();
end
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

%% Potential fields parameters tuning
[R_obstacles, U_0_alpha_B_obstacles]             = tune_potential(0.3, 0.5, 4, .2);
[R_agents,    U_0_alpha_B_agents]                = tune_potential(0.5, 0.8, 4, .2);
[R_obstacles_react, U_0_alpha_B_obstacles_react] = tune_potential(0.6, 0.8, 4,  2);
[R_vortex_react, U_0_alpha_B_vortex_react]       = tune_potential(0.6, 0.8, 4,  2);
[R_vortex, U_0_vortex]                           = tune_potential(0.3, 0.5, 5, .3);
%% Collect all parameters for control in a structure
ControlParameter.U_0_obstacles                = U_0_alpha_B_obstacles;
ControlParameter.R_obstacles                  = R_obstacles;
ControlParameter.U_0_obstacles_react          = U_0_alpha_B_obstacles_react;
ControlParameter.R_obstacles_react            = R_obstacles_react;
ControlParameter.R_vortex_react               = R_vortex_react;
ControlParameter.U_0_vortex_react             = U_0_alpha_B_vortex_react;
ControlParameter.U_0_agents                   = U_0_alpha_B_agents;
ControlParameter.R_agents                     = R_agents;
ControlParameter.R_vortex                     = R_vortex;
ControlParameter.U_0_vortex                   = U_0_vortex;
ControlParameter.k_a                          = 4; % to define attractive force
ControlParameter.safe_angle_big               = 40*pi/180;%40*pi/180;%10*pi/180%40*pi/180;
ControlParameter.safe_angle_small             = 30*pi/180;%30*pi/180%30*pi/180;
ControlParameter.v_des                        = 0.5;
ControlParameter.integral_gain_v_acceleration = 1;
ControlParameter.integral_gain_v_brake        = 5; % it was 20
ControlParameter.omega_gain                   = 4;%4;
ControlParameter.omega_power                  = 0.4;
ControlParameter.d_co_1_x                     = 1;
ControlParameter.d_co_2_x                     = 2;
ControlParameter.F_co_max_x                   = ControlParameter.k_a*1.1;
ControlParameter.d_co_1_y                     = 1;
ControlParameter.d_co_2_y                     = 2;
ControlParameter.F_co_max_y                   = ControlParameter.k_a*0.15;
ball_wp_radius                                = .3;
d_communication                               = 10^9; % range to define neighbors
communication_steps                           = 15;

%% START SIMULATION: Compute wp_path
[~, ~, wp_path,~,~] = generate_wp_path(waypoints, 0, 0, 1, 0);

h_waitbar = waitbar(0, 'Simulating...');
for kk = 1:(length(time)-1)
    %% Compute center of the formation
    %     if flag == 1
    %         break
    %     end
    switch center_algo_flag
        %%
        % The goal is to fill center matrix, a matrix, a matrix having two
        % columns and n_agents row. The i-th row is the [x,y] position of
        % the formation center estimated by agent i
        case 1
            %% basic algo
            center_matrix = zeros(n_agents,2);
            for j = 1:n_agents
                neighbors          = my_neighbors(x(kk,:), y(kk,:), theta(kk,:), j, d_communication);
                x_center           = mean([neighbors(1,:), x(kk,j)],2);
                y_center           = mean([neighbors(2,:), y(kk,j)],2);
                center_matrix(j,:) = [x_center, y_center];
            end
        case 2
            %% use matropolis mean
            center_matrix = metropolis_mean(x(kk,:), y(kk,:), communication_steps, d_communication);
    end
    % store matrix of the centers
    center_matrix_store(:,:,kk) = center_matrix;
    %% Loop on the agents
    for j = 1:n_agents
        
        %% Compute the current waypoint in a smart way
        [wp_input1, index_prev_vec(j), ~, Tend, flag] = generate_wp_path(waypoints, x(kk,j), y(kk,j), index_prev_vec(j), kk);
        dist_1 = norm(wp_input1 - [x(kk,j); y(kk,j)]);
        wp_input = wp_input1;
        wp_input_vec(:,kk) = wp_input;
        
        %% Specify the baricenter of the formation
        x_center = center_matrix(j,1);
        y_center = center_matrix(j,2);
        
        %% Initialize states of the controller
        if kk == 1
            v_old = 0;
            hysteresis_flag_old = 0;
        else
            v_old               = v(kk-1,j);
            hysteresis_flag_old = hysteresis_flag(kk-1,j);
        end
        %% Define velocities of obstacles
        v_obs(:,1,kk) = [-0.2+ 0.01*randn();0.01*randn()];
%         v_obs(:,2,kk) = [-0.5+0.01*randn();0.01*randn()];
%         v_obs(:,3,kk) = [-0.1+0.01*randn();-0.4+0.01*randn()];
%         v_obs(:,4,kk) = [0.1+0.01*randn();0.4+0.01*randn()];
% %         v_obs(:,5,kk) = [0;0];
%         
        
        %% Compute controls via SFM force
        neighbors                           = my_neighbors(x(kk,:), y(kk,:), theta(kk,:), j, d_communication); % compute my neighbors
        [F_total, F_permanent, F_transient] = compute_SFM_forces(neighbors, x(kk,j), y(kk,j), theta(kk,j), obstacle_points, ControlParameter, wp_input, x_center, y_center, wp_path);
        F_lc = 0;
        
        x_obstacle_equivalent(kk,:)  = x_obstacle(kk,:);
        y_obstacle_equivalent(kk,:)  = y_obstacle(kk,:);
        vx_obstacle_equivalent(kk,:) = v_obs(1,:,kk);
        vy_obstacle_equivalent(kk,:) = v_obs(2,:,kk);
        for q = 1:n_obstacles
            %% compute mean value of the permanent force inside the limit cycle
            F_permanent_mean = 0;
            points_mean = 1;
            for qq = 1:points_mean
                [~, F_permanent_tmp, ~] = compute_SFM_forces(neighbors, x_obstacle(kk,q) + cos(qq/points_mean*2*pi), y_obstacle(kk,q) + sin(qq/points_mean*2*pi), theta(kk,j), obstacle_points, ControlParameter, wp_input, x_center, y_center, wp_path);
                F_permanent_mean = F_permanent_mean + F_permanent_tmp;
            end
            %             F_permanent_mean    = F_permanent_mean/points_mean;
            %             [F_lc1, rot_matrix_store(kk, q, :, :),ControlParameter.omega_gain] = dynamic_obstacle_limit_cycles3(x(kk,j), y(kk,j), theta(kk,j), v_old, x_obstacle(kk,q), y_obstacle(kk,q), v_obs(1,q,kk), v_obs(2,q,kk), ControlParameter, F_permanent_mean, egg, waypoints,direction_prev(q));
            F_permanent_mean = 0;
            for qq = 1:points_mean
                [~, F_permanent_tmp, ~] = compute_SFM_forces(neighbors, x_obstacle_equivalent(kk,q) + cos(qq/points_mean*2*pi), y_obstacle_equivalent(kk,q) + sin(qq/points_mean*2*pi), theta(kk,j), obstacle_points, ControlParameter, wp_input, x_center, y_center, wp_path);
                F_permanent_mean = F_permanent_mean + F_permanent_tmp;
            end
            F_permanent_mean    = F_permanent_mean/points_mean;
            %             if sqrt((x(kk,j)-x_obstacle_equivalent(kk,q))^2 + (y(kk,j)-y_obstacle_equivalent(kk,q))^2) < 3 && count(q,j) == 0
            if count(q,j) == 0
                
                [F_lc2, rot_matrix_store(j, q, :, :),~, direction1(q,j)] = dynamic_obstacle_limit_cycles3(x(kk,j), y(kk,j), theta(kk,j), v_old, x_obstacle_equivalent(kk,q), y_obstacle_equivalent(kk,q), vx_obstacle_equivalent(kk,q), vy_obstacle_equivalent(kk,q), ControlParameter, F_permanent, egg, waypoints, F_max);
                if norm(F_lc2)>1e-3
                    count(q,j) = count(q,j) + 1;
                end
                F_lc                                    = F_lc  + F_lc2;
            elseif count(q,j) > 0
                %                 direction1(q,j);
                [F_lc2,~ ,~] = dynamic_obstacle_limit_cycles4(x(kk,j), y(kk,j), theta(kk,j), v_old, x_obstacle_equivalent(kk,q), y_obstacle_equivalent(kk,q), vx_obstacle_equivalent(kk,q), vy_obstacle_equivalent(kk,q), ControlParameter, F_permanent, egg, waypoints, direction1(q,j),rot_matrix_store(j, q, :, :), F_max);
                F_lc                                    = F_lc  + F_lc2;
            end
        end
        F_lc = min(.99*ControlParameter.k_a, max(-.99*ControlParameter.k_a, F_lc));
        F_total                                     = F_total     + F_lc;
        F_permanent                                 = F_permanent + 0*F_lc;
        % decides who has the control authority
        [flag_v, flag_omega, h_reference] = control_authority(theta(kk,j), ControlParameter, F_total, F_permanent, hysteresis_flag_old);
        hysteresis_flag(kk,j)             = flag_omega;
        h_reference_vec(:,kk,j)           = h_reference;
        % compute inputs of user and robot
        if kk == 1
            [v_robot, omega_robot] = control_robot(theta(kk,j), h_reference, v_old, dt, ControlParameter, h_reference_vec(:,kk,j));
        else
            [v_robot, omega_robot] = control_robot(theta(kk,j), h_reference, v_old, dt, ControlParameter, h_reference_vec(:,kk-1,j));
            
        end
        [v_user,  omega_user]  = control_user(v_old, dt, ControlParameter);
        
        %% Decide v and omega
        % v
        if flag_v == 0
            v(kk,j) = v_user;
        else
            v(kk,j) = v_robot;
        end
        % omega
        
        omega(kk,j) = omega_robot;
        
        %% save data
        F_total_vec(:,kk,j)     = F_total;
        F_permanent_vec(:,kk,j) = F_permanent;
        %% Simulate the dynamics
        % derivatives
        x_dot     = v(kk,j) * cos(theta(kk,j));
        y_dot     = v(kk,j) * sin(theta(kk,j));
        theta_dot = omega(kk,j);
        if norm([x(kk,j)-waypoints(1,end),y(kk,j)-waypoints(2,end)])<0.1
            x_dot = 0;
            y_dot = 0;
            theta_dot =0;
        end
        % Euler integration
        x(kk+1,j)     = x(kk,j)     + x_dot     * dt;
        y(kk+1,j)     = y(kk,j)     + y_dot     * dt;
        theta(kk+1,j) = theta(kk,j) + theta_dot * dt;
    end
    waitbar(kk/(length(time)-1));
    %% Define the trajectory of the dynamic obstacle
    for q = 1:n_obstacles
        x_obstacle(kk+1,q) =  x_obstacle(kk,q) + dt*v_obs(1,q,kk);
        %     x_obstacle(kk+1) = 306;
        y_obstacle(kk+1,q) =  y_obstacle(kk,q) + dt*v_obs(2,q,kk);
    end
    

end
close(h_waitbar);
%% Post processing
figure('Name', 'Animation','units','normalized','outerposition',[0 0 1 1]);
hold on;

% title('Map','Interpreter','latex');
xlabel('x$_{\textrm{glob}}$ [m] ','Interpreter','latex');
ylabel('y$_{\textrm{glob}}$ [m] ','Interpreter','latex');
axis equal;
% fig_1_ObstPoly = plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis([-2 12 3 7])
axis off
% plot waypoints
% for i = 1:n_waypoints
    plot(waypoints(1,end), waypoints(2,end), 'x', 'markersize', 15, 'color', 'r','linewidth',2);
% end
scal = 0.4;
for kk = 1:1:Tend %length(time-1)-1
    
    plot_obj   = gobjects(n_agents,6); % initialize array of plots
    cone_obj   = gobjects(n_agents,4); % initialize array of plots
    arrow1     = gobjects(n_agents,1);
    arrow2     = gobjects(n_agents,1);
    arrow3     = gobjects(n_agents,1);
    set(gcf, 'color', [1 1 1])
    center_obj = gobjects(n_agents,2);
    obstacle   = gobjects(n_obstacles,12);
    obs_circle = gobjects(n_obstacles,n_agents);
    set(gcf, 'Color', 'k')

    % plot real center of the formation
%     center = plot(mean(x(kk,:)), mean(y(kk,:)), 's', 'markersize', 8, 'color', 'c');
    for j = 1:n_agents
        % Plot forces
        
        if kk ~= length(time) && norm([x(kk,j)-waypoints(1,end),y(kk,j)-waypoints(2,end)])>0.1

            arrow1(j) = quiver(x(kk,j), y(kk,j), F_total_vec(1,kk,j)*scal,  scal*F_total_vec(2,kk,j),  'color', 'r', 'linewidth', 2.3, 'linestyle', ':');
            arrow2(j) = quiver(x(kk,j), y(kk,j), F_permanent_vec(1,kk,j)*scal, scal*F_permanent_vec(2,kk,j), 'color', 'g', 'linewidth', 1.7);
            arrow3(j) = quiver(x(kk,j), y(kk,j), (F_total_vec(1,kk,j)-F_permanent_vec(1,kk,j))*scal, (F_total_vec(2,kk,j)-F_permanent_vec(2,kk,j))*scal, 'color', 'b', 'linewidth', 1.7);

        end
        % plot trajectories
        plot(x(1:kk,j), y(1:kk,j), 'linewidth', 1.5, 'color', [1 0.7 0], 'linestyle',':');
        % plot robots
        plot_obj(j,:) = plot_unicycle(x(kk,j), y(kk,j), theta(kk,j), 'w');
        % plot cones
        %         cone_obj(j,:) = plot_cone(x(kk,j), y(kk,j), h_reference_vec(:,kk,j), ControlParameter.safe_angle_big, ControlParameter.safe_angle_small);
        % plot active wp
        %         active_wp =  plot(wp_input_vec(1,kk), wp_input_vec(2,kk), 's', 'markersize', 8, 'color', 'c');
        % plot center of the formation
%         x_center        = center_matrix_store(j,1,kk);
%         y_center        = center_matrix_store(j,2,kk);
%         center_obj(j,1) = plot(x_center, y_center, 'x', 'markersize', 8, 'color', 'b');
%         center_obj(j,2) = plot([x_center x(kk,j)], [y_center y(kk,j)], 'linewidth', 1.0, 'color', [0.7 0.7 0.7], 'linestyle','--');
        
    end
    
    %     if limit_cycle_shape == 0
    %         % plot the obstacle
    %         for q = 1:n_obstacles
    %             obstacle(q)   = plot(x_obstacle(kk,q), y_obstacle(kk,q), '*', 'markersize', 12, 'color', 'k');
    %             obs_circle(q) = plot(x_obstacle(kk,q) + cos(0:0.1:2*pi), y_obstacle(kk,q) + sin(0:0.1:2*pi), 'linewidth', 1.0, 'color', [0.1 0.1 0.7], 'linestyle','--');
    %         end
    %     else
    for q = 1:n_obstacles
        obstacle(q,:) = human(x_obstacle(kk,q), y_obstacle(kk,q),v_obs(1,q,kk), v_obs(2,q,kk));
    end
    for j = 1:n_agents
        
        for q = 1:n_obstacles % plot eggs
            %           obstacle(q,:)   = plot(x_obstacle(kk,q), y_obstacle(kk,q), '*', 'markersize', 12, 'color', 'k');
            x_egg_plot    = egg.a*cos(0:0.1:2*pi);
            y_egg_plot    = egg.b*exp(-egg.alpha*(egg.a*cos(0:0.1:2*pi))/2).*sin(0:0.1:2*pi);
            %         egg.aa = 2.5;
            %         egg.bb = egg.a*0.7;
            %         x_egg_plot1   = egg.aa*cos(0:0.1:2*pi);
            %         y_egg_plot1   = egg.b*exp(-egg.alpha*(egg.a*cos(0:0.1:2*pi))/2).*sin(0:0.1:2*pi);
            %         y_egg_plot1   = egg.bb*exp(-egg.alpha*(egg.aa*cos(0:0.1:2*pi))/2).*sin(0:0.1:2*pi);
            for qq = 1:length(x_egg_plot)
                tmp_egg        = [rot_matrix_store(j, q, 1, 1) rot_matrix_store(j, q, 1, 2); rot_matrix_store(j, q, 2, 1) rot_matrix_store(j, q, 2, 2)] * [x_egg_plot(qq) + egg.trasl; y_egg_plot(qq)];
                x_egg_plot(qq) = tmp_egg(1);
                y_egg_plot(qq) = tmp_egg(2);
                %             tmp_egg1        = [rot_matrix_store1(kk, q, 1, 1) rot_matrix_store1(kk, q, 1, 2); rot_matrix_store1(kk, q, 2, 1) rot_matrix_store1(kk, q, 2, 2)] * [x_egg_plot1(qq) + egg.trasl; y_egg_plot1(qq)];
                %             x_egg_plot1(qq) = tmp_egg1(1);
                %             y_egg_plot1(qq) = tmp_egg1(2);
            end

            obs_circle(j,q) = plot(x_obstacle(kk,q) + x_egg_plot , y_obstacle(kk,q) + y_egg_plot, 'linewidth', 1.0, 'color', [1 1 0], 'linestyle','--');
        end
    end
    
    if video_flag ==1
%         axis([x(kk,1)-4 x(kk,1)+4 y(kk,1)-4 y(kk,1)+4] )
        drawnow
        F(kk) = getframe(gcf); %#ok<*SAGROW>
    else
        drawnow
    end
    %         pause
    delete(arrow1);
    delete(arrow2);
    delete(arrow3);
    delete(plot_obj);
    delete(cone_obj);
%     delete(center);
    delete(center_obj);
    delete(obstacle);
    %delete(active_wp);
    delete(obs_circle);
    %delete(obs_circle1);
    
end
%% Create a file .avi of simulation results
if video_flag == 1
    video = VideoWriter('simulazione1.avi','Motion JPEG AVI');
    video.Quality = 80;
    video.FrameRate = 1/dt;
    open(video)
    writeVideo(video,F(1:224))
    close(video)
end