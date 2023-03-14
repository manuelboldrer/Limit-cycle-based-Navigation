function [point, flag] = line_segment_intersec(line, segment)
% line = [m;q], the y = m*x+q
% segment = [x1 x2 y1 y2]
% flag = the intersection is inside the segment (i.e., flag = 1, else
% flag = 0)
% point = [xp;yp], intersection between the two
%%
x_1 = segment(1);
y_1 = segment(3);
x_2 = segment(2);
y_2 = segment(4);
m_l = line(1);
q_l = line(2);

%% Compute the straight line containing segment
% y - y0  = m*(x - x0) -> y  = m * x + y0-m*x0,
% y = m*x + q, q = + y0-m*x0
%% Face singularity of m = inf
if abs(x_2 - x_1) > 10^(-2)
    angle = 0;
else %% rotate reference frame to avoid singularities
    angle = pi/2;
    % redefine m_l, q_l
end
%% Write new points in the frame
R   = [cos(angle) -sin(angle); sin(angle) cos(angle)];
tmp = R * [x_1; y_1];
x_1 = tmp(1);
y_1 = tmp(2);
tmp = R * [x_2; y_2];
x_2 = tmp(1);
y_2 = tmp(2);
% redefine m_l, q_l
p_1 = [0; q_l]; % point of the line corresponding to x = 0
p_2 = [1; m_l + q_l]; % point of the line corresponding to x = 1
p_1 = R * [p_1(1); p_1(2)];
p_2 = R * [p_2(1); p_2(2)];
m_s = (y_2 - y_1)/(x_2 - x_1); % m segment
q_s = y_1 - m_s * x_1;         % q segment
%% Compute the point
if abs(p_2(1) - p_1(1)) < 10^(-2)
    % the line is vertical, its equation is x = const = p_1(1)
    x_intersec = (p_2(1) + p_1(1))/2;
    y_intersec = m_s * x_intersec + q_s;
else % the line is not vertical
    m_l = (p_2(2) - p_1(2)) / (p_2(1) - p_1(1));
    q_l = p_1(2) - m_l * p_1(1);
    %% Compute intersection
    x_intersec = -(q_l - q_s)/(m_l - m_s);
    y_intersec = m_l * x_intersec + q_l;
end
%% Now check if the intersection is inside the segment
x_max = max(x_1, x_2);
x_min = min(x_1, x_2);
if x_intersec > x_max || x_intersec < x_min
    flag = 0; % no intersection
else
    flag = 1; % intersection
end
point = [x_intersec; y_intersec];
point = R' * point;% return in the proper coordinate system
