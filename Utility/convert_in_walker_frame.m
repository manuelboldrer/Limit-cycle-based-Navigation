function [angle_wf, distance_wf] = convert_in_walker_frame(angle, distance, theta_walker)
% convert the output of lidar acquisition in the walker frame
angle_wf    = zeros(length(angle),1);
distance_wf = zeros(length(angle),1);

for i = 1:length(angle)
    if isnan(distance(i)) || isinf(distance(i))
        x_lf = cos(angle(i)); % x of the point in the lidar frame
        y_lf = sin(angle(i)); % y of the point in the lidar frame
        R    = [cos(theta_walker), -sin(theta_walker); sin(theta_walker), cos(theta_walker)]; % rotation matrix from walker frame to lidar frame
        tmp  = R'*[x_lf; y_lf];
        x_wf = tmp(1); % x of the point in the waalker frame
        y_wf = tmp(2); % y of the point in the waalker frame
        % now convert in polar coordinates
        angle_wf(i)    = atan2(y_wf, x_wf);
        distance_wf(i) = inf;
    else
        x_lf = distance(i) * cos(angle(i)); % x of the point in the lidar frame
        y_lf = distance(i) * sin(angle(i)); % y of the point in the lidar frame
        R    = [cos(theta_walker), -sin(theta_walker); sin(theta_walker), cos(theta_walker)]; % rotation matrix from walker frame to lidar frame
        tmp  = R'*[x_lf; y_lf];
        x_wf = tmp(1); % x of the point in the waalker frame
        y_wf = tmp(2); % y of the point in the waalker frame
        % now convert in polar coordinates
        angle_wf(i)    = atan2(y_wf, x_wf);
        distance_wf(i) = sqrt(x_wf^2 + y_wf^2);
    end
end