function [wp_actual, points] = wp2rectangle(wp, wp_old, LidarScanArea, obstaclesTree, x, y)
%% Parameters
alpha  = .4; % distance side rectangle from circle
n_disc = 15;
alpha2 = .4; % min distance of a wp on the rectangle to the obstacles
%%
x_wp  = wp(1);
y_wp  = wp(2);

%% Find the points of the obstacles
lidarScan = simulateLIDAR(LidarScanArea, obstaclesTree, [x_wp; y_wp; 0]);
%% Generate angles from lidar scan
angles = linspace(lidarScan.AngleMin,lidarScan.AngleMax,numel(lidarScan.Ranges));
range  = lidarScan.Ranges;
%% Find the closest point of the closest obstacle to the wp
min_range = range(1);
index_min = 1;
for i = 2:length(angles)
    if lidarScan.Ranges(i) < min_range
        min_range = range(i);
        index_min = i;
    end
end
closest_obst = min_range * [cos(angles(index_min)); sin(angles(index_min))] + [x_wp; y_wp]; % point of obstacle closest to the wp
%% Define the rectangle points in the reference frame centered in the wp
y_top    = min_range - alpha; % if this is negative you should return wp
if y_top <= 0
    wp_actual = [x_wp; y_wp];
    points    = zeros(4);
    return;
end
y_bottom = -y_top;
x_right  = sqrt(min_range^2 - y_top^2);
x_left   = -x_right;

points = [[x_right; y_top], [x_left; y_top], [x_left; y_bottom], [x_right; y_bottom]];



wp_old2wp  = (wp - wp_old)/norm(wp - wp_old); % unit vector from wp_old to wp
angle      = atan2(-wp_old2wp(2),-wp_old2wp(1));
Rot_matrix = [cos(angle) -sin(angle); sin(angle), cos(angle)];
for i = 1:size(points,2)
    points(:,i) = Rot_matrix * points(:,i) + wp;
end

%% Discretize each side of the rectangle
rect_points = zeros(2, n_disc * 4);
inserted_points = 0;
for i = 1:4 % for each side of the rectangle
    P1          = points(:,i);
    P2          = points(:, 1 + mod(i-1 + 1, size(points,2)));
    L           = norm(P1 - P2);
    s           = linspace(0, L, n_disc);
    for j = 1:length(s) % generate n_disc points
        inserted_points                = inserted_points + 1;
        rect_points(:,inserted_points) = P1 + (P2-P1)*s(j)/L;
    end
end

%% Check if the points in the rectangle are too close to the obstacle
points_ok_counter = 0;
for i = 1:size(rect_points,2)
   if norm(closest_obst - rect_points(:,i)) >= alpha2
       points_ok_counter = points_ok_counter + 1;
   end
end
% if all the points are too close to the obstacles return the tru wp
if points_ok_counter == 0
    wp_actual = [x_wp; y_wp];
    return;
end
    
rect_points_ok  = zeros(2, points_ok_counter);
inserted_points = 0;
for i = 1:size(rect_points,2)
   if norm(closest_obst - rect_points(:,i)) >= alpha2
       inserted_points                    = inserted_points + 1;
       rect_points_ok(:, inserted_points) = rect_points(:,i);
   end
end
%% Find the closest point of the rectangle to the vehicle

min_distance = norm(rect_points_ok(:,1) - [x;y]);
index_min    = 1;
for i = 2:size(rect_points_ok, 2)
    if min_distance > norm(rect_points_ok(:,i) - [x;y])
        min_distance = norm(rect_points_ok(:,i) - [x;y]);
        index_min    = i;
    end
end
%%
wp_actual = rect_points_ok(:,index_min);


