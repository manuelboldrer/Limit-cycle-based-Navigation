function [F_lc, rot_matrix, omega_gain] = dynamic_obstacle_limit_cycles4(x_robot, y_robot, theta_robot, v_robot, x_obstacle, y_obstacle, v_x_obstacle, v_y_obstacle, params, F_permanent, egg, waypoints,direction, rot_matrix, F_max1)
%% Parameter limit cycle
% F_max1    = 2*params.k_a; % max force applied by the limit cycle (in the obstacle)
%F_max2    = 5*params.k_a; 
F_min     = .0  ; % max force applied by the limit cycle (in the circle)
K_max     = 4   ; %13; % omega gain max (close to the obstacle)
K_min     = 4   ; % omega gain min (far away from the obstacle)
alpha     = 1   ; % weight to establish the direction of the cycle
beta      = 1   ; % weight to establish the direction of the cycle
gamma     = 2.5   ; %1.45;%2.2;%1.5; % weight to establish the direction of the cycle
v_bar     = 0.5 ; % all the obstacles faster than v_bar are treated at the same time
phi       = 10  ; %50;%5; %increase if the obstacle's dimensions increase! (potenzialmente è diverso per ogni ostacolo)
psi       = .5;%.5  ; %0;
ro        =  1  ;
rot_matrix = [rot_matrix(:,:,1,1), rot_matrix(:,:,1,2); rot_matrix(:,:,2,1),rot_matrix(:,:,2,2)];

%% Define the orientation of the egg on the basis of the waypoints
% get the wp path
[~, ~, wp_path,~,~] = generate_wp_path(waypoints, x_obstacle, y_obstacle, 1,0);
% find the two closest points of the wp path to the obstacle and the
% corresponding indexes

if norm(wp_path(:,1) - [x_obstacle; y_obstacle]) > norm(wp_path(:,2) - [x_obstacle; y_obstacle])
    small_min   = norm(wp_path(:,2) - [x_obstacle; y_obstacle]);
    big_min     = norm(wp_path(:,1) - [x_obstacle; y_obstacle]);
    index_small = 2;
    index_big   = 1;
else
    small_min   = norm(wp_path(:,1) - [x_obstacle; y_obstacle]);
    big_min     = norm(wp_path(:,2) - [x_obstacle; y_obstacle]);
    index_small = 1;
    index_big   = 2;
end

for i = 3:size(wp_path,2)
    dist_tmp = norm(wp_path(:,i) - [x_obstacle; y_obstacle]) ;
    if dist_tmp < small_min
        big_min     = small_min;
        index_big   = index_small;
        small_min   = dist_tmp;
        index_small = i;
    elseif dist_tmp < big_min
        big_min     = dist_tmp;
        index_big   = i;
    end
end

% now define the direction of the egg
index_1 = min(index_big,index_small);
index_2 = max(index_big,index_small);
h_egg   = (wp_path(:,index_1) - wp_path(:,index_2)) / norm(wp_path(:,index_1) - wp_path(:,index_2)); % here the egg is oriented as the permanent field
%h_egg  = [1;0];
%% Obstacle velocity 
v_obs   = [v_x_obstacle;v_y_obstacle];
% have a look on the moving direction of the obstacle
if norm(v_obs) > 0.01 
    h_v_obs     = v_obs/norm(v_obs); % direction of the velocity of the obstacle
else
    h_v_obs     = h_egg; % i.e., if the obstacle is still, I think it is moving as the permanent field
end
%% Compute the heading of the vehicle
h = [cos(theta_robot); sin(theta_robot)];
%% Parameters for the egg

% if flag_cl >0
%     a         = 2.5;
% %     egg.trasl = a/2;
%     epsilon   = 0.7;%egg.epsilon;
% else
%     a         = egg.a;
% %     egg.trasl = a/2;
%     epsilon   = 0.5;
% end
a         = egg.a;
epsilon   = egg.epsilon;
b         = a*epsilon;
alpha_egg = egg.alpha;
x_c       = -egg.trasl;


%% Choose the direction
pos_robot_0 = [x_robot; y_robot]; % position of the robot in ref frame 0 (ground)
% compute h_l_plus
h_plus = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)] * (pos_robot_0 - [x_obstacle; y_obstacle]) / norm(pos_robot_0 - [x_obstacle; y_obstacle]); % counter clock wise, i.e., direction = +1
% compute the heading of the field h_u
h_u = F_permanent/norm(F_permanent); % wrong

% chose direction
n_RO  = [x_obstacle - x_robot; y_obstacle - y_robot]/norm([x_obstacle - x_robot; y_obstacle - y_robot]); % this unit vector points from robot to obstacle
v_RO  = dot(n_RO, v_robot * h - v_obs); %#ok<NASGU> % relative velocity from robot to obstacle; negative if the robot is moving away
% direction = sign(alpha * dot(h_plus, h_u) + beta * dot(h_plus, h) - gamma * sign(dot(h_plus, v_obs))* g_rel(v_bar, v_RO) * g(v_bar, v_obs) );


%     direction = sign(alpha * dot(h_plus, h_u) + beta * dot(h_plus, h) - gamma * sign(dot(h_plus, v_obs)) * g2(cos(45*pi/180), h, h_v_obs) * g(v_bar, v_obs) ) ;


% if dot(direction * h_plus, h_u) < -0.5 && dot(h, pos_robot_0 - [x_obstacle; y_obstacle]) < 0 % then you are in the wrong side of the egg and you are getting closer to the obs
%     rot_matrix  = [h_egg, [-h_egg(2); h_egg(1)]]; % from RF1 to RF0
%     pos_robot_1 = rot_matrix' * (pos_robot_0 - [x_obstacle; y_obstacle]); % position of the robot in ref frame 1
%     x           = pos_robot_1(1);
%     y           = pos_robot_1(2);
%     angle       = 1.9*atan2(y, x);%1.1
%     if abs(angle)<pi/2
%         h_egg       = [cos(angle) -sin(angle); sin(angle) cos(angle)] * h_egg;
%     end
% end

% if dot(h, h_v_obs) > 0.8
%     h_egg = -h_egg; % this may help overturn maneuvers
% end
%% Define the vector of the limit cycle
% put the robot in the reference frame of the obstacle translated by a/2
% we have 3 reference frames:
% 0 = gorund
% 1 = centered in the obstacle and oriented as the permanent field
% 2 = traslated in the direction of the obstacle velocity by a/2
% rot_matrix  = [h_egg, [-h_egg(2); h_egg(1)]]; % from RF1 to RF0
pos_robot_1 = rot_matrix' * (pos_robot_0 - [x_obstacle; y_obstacle]); % position of the robot in ref frame 1
pos_robot_2 = pos_robot_1 - [egg.trasl; 0];
x           = pos_robot_2(1);
y           = pos_robot_2(2);
% I will use x, y to compute everithing. They are the coordinates of the
% robot in the frame 2

%% Compute the vector field

r        =   ((epsilon * x * x_c) - (epsilon * x_c ^ 2) + sqrt((a ^ 2 * epsilon ^ 2 * x ^ 2) - (2 * a ^ 2 * epsilon ^ 2 * x * x_c) + (a ^ 2 * epsilon ^ 2 * x_c ^ 2) + y ^ 2 * exp((alpha_egg * x)) * (a ^ 2) - exp((alpha_egg * x)) * (x_c ^ 2) * y ^ 2)) / (a ^ 2 - x_c ^ 2) * a / epsilon;
d        = x_c * (1-r/a);
x_bar    = x - d;
x_vec_ff = - a / b * y * exp(alpha_egg* (x_bar) / 2) ;
y_vec_ff = b / a * (x_bar) * exp(-alpha_egg* x_bar / 2) + y^2 * exp(alpha_egg* x_bar / 2) * alpha_egg * a / 2 / b; %% TO BE VERIFIED
vec_fb   = 1 - (x/a)^2 - (y/b)^2 * exp(alpha_egg * x);
x_vec    = ro*direction * x_vec_ff + psi * (x-x_c) * vec_fb;
y_vec    = ro*direction * y_vec_ff + phi * y * vec_fb;



% end
%% Scale the field with the distance
velocity         = [x_vec; y_vec];
vel_heading      = velocity/norm(velocity);
% if limit_cycle_shape == 0
%     dist_2_center = norm([x;y]);
%     if dist_2_center >= 1
%         force_norm = 0;
%     else
%         force_norm = F_max + (F_max - F_min)/(-1) * (dist_2_center);
%     end
% else
    dist_2_center = x^2/a^2 + y^2/b^2*exp(alpha_egg*(x)); % check if you are inside the front egg
    dist_2_obst   = x_bar^2/a^2 + y^2/b^2*exp(alpha_egg*(x_bar));

    if dist_2_center > 1
        force_norm  = 0;
        vel_heading = [1;0];
        omega_gain  = K_min;
    else 
        force_norm = F_max1 - (F_max1 - F_min)* (dist_2_obst); % check if its correct!
        omega_gain = K_max - (K_max - K_min)* (dist_2_obst); 
%     else
%         force_norm = F_max2 - (F_max2 - F_max1)* (dist_2_obst);
    end
    
% end
    
   F_lc = rot_matrix * force_norm * vel_heading;
end

function out = g_rel(v_bar, v_RO) %#ok<DEFNU>
if (v_RO) <= 0
    out = 0;
elseif (v_RO) <= v_bar
    out = 1/v_bar * norm(v_RO);
else
    out = 1;
end
end

function out = g(v_bar, v_obs)
if norm(v_obs) <= v_bar
    out = 1/v_bar * norm(v_obs);
else
    out = 1;
end
end
   
function out = g2(threshold, h, h_obs)
% threshold is an angle. The more the velocities of robot and osbtacle are
% otrthogonal, the more the velocity is important in the choice of the
% direction of rotation
scalar_prod = dot(h, h_obs);
if scalar_prod <= threshold
    out = 1;
else
    out = 1 + (1 - 0) / (threshold - 1) * (scalar_prod - threshold);
end

end
