function [angle, measured_distance] = lidar_acquisition(lidar_pos, obs_cell, max_radius, angle_increment)
% This finction simulate the acquired data by a lidar located in lidar_pos.
% The outputs are the polar cooridinates of the obstacles, expressed in a
% reference frame centered in the lidar and alligned with the reference
% frame where the obstacle are expressed

angle           = 0:angle_increment:2*pi';
measured_distance = zeros(length(angle),1);
for i = 1:length(angle)
    line = [lidar_pos(1), lidar_pos(1) + cos(angle(i)), lidar_pos(2) lidar_pos(2) + sin(angle(i))];
    founded_points = [];
    for j = 1:length(obs_cell)
        segment        = obs_cell{j};
        [point, flag_intersec, flag_parall, flag_visible] = line_segment_intersec2(line, segment);
        if flag_parall == 0 && flag_intersec == 1 && flag_visible == 1 % than add the point easy
            
            founded_points = [founded_points, point]; % x y of the point written in column... add new column with two elementws per point
            
        elseif flag_parall == 2 && flag_visible == 1
            % they are parallel and the segment is contained in the line.
            % The lidar finds the closest extremum
            
            dist_1 = norm([segment(1); segment(3)] - lidar_pos);
            dist_2 = norm([segment(2); segment(4)] - lidar_pos);
            try
            if dist_1 <= dist_2
                founded_points = [founded_points, [segment(1); segment(3)]];
            else
                founded_points = [founded_points, [segment(2); segment(4)]];
            end
            catch
                keyboard 
            end
        else % (you can comment this else)
            % do nothing in the following cases:
            % - flag_intersec == 0  --> they do not touch because the intersection between lines is outside the segment -> do nothing
            % - flag_parall   == 1  --> they do not touch because parallel -> do nothing
            % - flag_visible  == -1 --> the intersection is defined, but it is contained in the vector started from point lidar_pos going to [lidar_pos(1) + cos(angle(i)), lidar_pos(2) + sin(angle(i))]
        end
    end
    % now take the closest point between the one founded.
    [~,n_columns] = size(founded_points);
    if n_columns == 1
        closest_pt = [founded_points(1,1); founded_points(2,1)];
    elseif n_columns > 1
        closest_pt = [founded_points(1,1); founded_points(2,1)];
        dist_min   = norm(closest_pt - lidar_pos);
        for kk = 2:n_columns % move on founded points
            point_kk = [founded_points(1,kk); founded_points(2,kk)];
            if norm(point_kk - lidar_pos) < dist_min
                dist_min   = norm(point_kk - lidar_pos);
                closest_pt = point_kk;
            end
        end
        
    else
        closest_pt = [inf; inf];
    end
    measured_distance(i) = norm(closest_pt - lidar_pos);
    if measured_distance(i) > max_radius
       measured_distance(i) = inf; 
    end
end






