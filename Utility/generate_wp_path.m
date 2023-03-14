function [wp_smart, index_min, wp_path, Tend, flag] = generate_wp_path(waypoints, x, y, index_prev, kk)
flag         = 0;
L_calculus   = 0.6;%3; % the largest curvilinear distance from the wp, i.e., max preview lenght
L_min        = 0.6; % minimum preview lenght
% max_distance = .3; % if the closest wp is further than this, return it as wp_smart
angle_max    = pi/3; % max tolerated angle to compute varying preview
 Tend = kk;
%%
distance_wp_path     = .01;  % a wp per distance_wp_path meters
length_window_search = 1.5; % meters around which I look for the minimum
n_waypoints_main = size(waypoints,2); % munber of wp inserted manually
n_sub_path       = n_waypoints_main - 1; % number of subpath
length_sub_path  = zeros(n_sub_path,1); % lenght of each piece of the path
n_wp_sub_path    = zeros(n_sub_path,1); % number of wp per sub path
for i = 1:length(n_wp_sub_path)    
    length_sub_path(i) = norm(waypoints(:,i+1) - waypoints(:,i));
    n_wp_sub_path(i)   = floor(length_sub_path(i) / distance_wp_path);
end
wp_path = zeros(2,sum(n_wp_sub_path));
%% Fill the subpath
inserted_wp = 0;
for i = 1:n_sub_path % move on the subpath
    P1 = waypoints(:,i);
    P2 = waypoints(:,i+1);
    s  = 0;
    for j = 1:n_wp_sub_path(i)
        P                      = P1 + s * (P2 - P1) / norm(P2 - P1);
        inserted_wp            = inserted_wp + 1;
        wp_path(:,inserted_wp) = P;
        s                      = min(s + distance_wp_path, length_sub_path(i));
    end
end
%% Compute the closest point of the path to me
min_dist   = norm(wp_path(:,index_prev) - [x;y]);
index_min  = index_prev;
for i = index_prev:min(size(wp_path,2),index_prev + floor(length_window_search/distance_wp_path))% 2:size(wp_path,2)
    dist = norm(wp_path(:,i)- [x;y]);
    if min_dist > dist
        min_dist   = dist;
        index_min  = i;
    end
end
% 
% if min_dist > max_distance
%     index_wp_smart = min(floor(index_min + 0.5*L_min/distance_wp_path), size(wp_path,2));
%     wp_smart       = wp_path(:, index_wp_smart);
%     return;
% end

%% Compute the length of the preview depending on the path curvature

% I will process the wp comprised in the following indexes:
index_wp_far   = min(floor(index_min + L_calculus/distance_wp_path), size(wp_path,2));
% index_wp_close = min(floor(index_min + L_min/distance_wp_path), size(wp_path,2));
index_wp_close = min(index_min, size(wp_path,2));

n_points_processed = index_wp_far - index_wp_close + 1;
unit_vec_store     = zeros(2,n_points_processed - 1); % each column is a unit vector from a wp to the following one
% Compute unit vectors
for i = 1:(n_points_processed-1)
    unit_vec_store(:, i) = wp_path(:, i + index_wp_close) - wp_path(:, i - 1 + index_wp_close);
    unit_vec_store(:, i) = unit_vec_store(:, i)/norm(unit_vec_store(:, i));
end
% Compute the increments of angles
angles_vec = zeros(n_points_processed - 1, 1); % the element i-th is the angle traveled from the beginning to wp i-th
for i = 2:length(angles_vec)
    %% Compute the dot prod between each unit vec and the following and augment the angle
    angles_vec(i) = angles_vec(i-1) + abs(acos(dot(unit_vec_store(:, i),unit_vec_store(:, i-1))));
end
i = length(angles_vec);
if i == 0
    flag = 1;
    wp_smart       = wp_path(:, index_prev);
    return
    %      error('SIMULAZIONE TERMINATA, kk = %d', kk)
end
 
while angles_vec(i) > angle_max
    i = i - 1;
end
index_wp_smart = i + index_wp_close - 1;

% Compute the curvilinear distance from wp_smart
distance = 0;
for i = index_min:(index_wp_smart - 1)
    distance = distance + norm( wp_path(:,i+1) - wp_path(:,i));
end

if distance < L_min
    index_wp_smart = min(floor(index_min + L_min/distance_wp_path), size(wp_path,2));
end
wp_smart = wp_path(:, index_wp_smart); % this is the wp_smart with the variable preview in the standard case

%%
% index_wp_smart = min(floor(index_min + length_preview/distance_wp_path), size(wp_path,2));
% wp_smart       = wp_path(:, index_wp_smart);
