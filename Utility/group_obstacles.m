function output = group_obstacles(angle, distance, tolerance)
% Given the vector of angles and distaces measured by the lidar, return a
% cell of obstacles. Each element of the cell is an array of points


coder.varsize('obstacle_vec', [], [1 0]);
coder.varsize('obst_group');
obstacle_vec = ones(0,1);
obst_group = cell(1,0);
n_obs        = 0; % number of obstacles
toll         = tolerance; % two points are in the same obs if they are are closer than toll
for i = 1:length(angle)
    r = distance(i);
    if isempty(obstacle_vec) && (not(isnan(r) || isinf(r))) % then start to fill an obstacle        
        obstacle_vec = i; % point i in the first obstacle
        n_obs = n_obs + 1; % there is a new obs
    elseif not(isempty(obstacle_vec)) % i.e., I am already filling an obstacle
        if point_dist_polar(obstacle_vec(end), i, angle, distance) < toll && not(isnan(r) || isinf(r))
            %% continue to add to the obstacle
            obstacle_vec = [obstacle_vec; i]; 
        elseif not(isnan(r) || isinf(r)) % then a new obs is needed
            obst_group{end+1}        = obstacle_vec;
            obstacle_vec      = i;
            n_obs             = n_obs + 1;
        elseif isnan(r) || isinf(r)% since not(isempty(obstacle_vec)) is true, I add the current non empty obs to the list
            obst_group{end+1}        = obstacle_vec;
            obstacle_vec      = ones(0,1);
        end
    end
end
%% add the last obstacle
if n_obs > 0 && not(isnan(distance(end)) || isinf(distance(end)))
    obst_group{end+1}        =  obstacle_vec;
end

%% now group obstacles (just first and last for now)

if n_obs > 0 && point_dist_polar(obst_group{1}(1), obst_group{end}(end), angle, distance) < toll
    obst_group{1} = [obst_group{1}; obst_group{end}];
    indice_finale = numel(obst_group)-1;
else
    indice_finale = numel(obst_group);    
end
%% Delete obstacles with one point
coder.varsize('output');
output = cell(1,0);
n_obs          = 0;
for i = 1:indice_finale
   if length(obst_group{i}) > 1
       n_obs             = n_obs + 1;
       output{end+1}        = obst_group{i};
   end
end



end
function dist = point_dist_polar(i,j,angle,distance)
x_i  = distance(i) * cos(angle(i));
y_i  = distance(i) * sin(angle(i));
x_j  = distance(j) * cos(angle(j));
y_j  = distance(j) * sin(angle(j));
if isinf(distance(i)) || isinf(distance(j)) || isnan(distance(i)) || isnan(distance(j))
    dist = inf;
else
    dist = norm([x_i - x_j; y_i - y_j]);
end
end