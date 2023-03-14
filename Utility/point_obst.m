function points = point_obst(obstacle_cell, angle, distance, walker_pos)
% given a cell of obstacle, such that each element of the cell is an array
% of points of the obstacle, this funtion returns the closest point for
% each obstacle

points = zeros(2,length(obstacle_cell)); % each column is a point

for i = 1:length(obstacle_cell)
    obs_vec      = obstacle_cell{i}; % this is an obstacle defined as array of points
    closest_pt   = obs_vec(1);
    min_distance = walker2point_dist(closest_pt, angle, distance, walker_pos);
    for j = 2:length(obs_vec)
        tmp = walker2point_dist(obs_vec(j), angle, distance, walker_pos);
        if min_distance > tmp;
            min_distance = tmp;
            closest_pt   = obs_vec(j);
        end
    end
    points(:,i) = [distance(closest_pt) * cos(angle(closest_pt)); distance(closest_pt) * sin(angle(closest_pt))];
    
end

end


function dist = walker2point_dist(index, angle, distance, walker_pos)
if isinf(distance(index)) || isnan(distance(index))
    dist = inf;
else
    x    = distance(index) * cos(angle(index));
    y    = distance(index) * sin(angle(index));
    dist = norm([x - walker_pos(1); y - walker_pos(2)]);
end

end