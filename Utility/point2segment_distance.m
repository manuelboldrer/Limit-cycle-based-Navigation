function [p_cl,d] = point2segment_distance(segment, point)
% compute the distance d between point and segment
% segment in this way: [x_1 y_1 x_2 y_2]
% point = [x_p; y_p]
% p_cl = point of the segment closest to the input point
%% Compute the straight line containing segment
% y - y0  = m*(x - x0) -> y  = m * x + y0-m*x0,
% y = m*x + q, q = + y0-m*x0
if abs(segment(3)- segment(1)) > 10^(-2)
    m = (segment(4) - segment(2))/(segment(3)- segment(1));
    q = segment(2) - m * segment(1);
    % d = abs(point(2) - m*point(1) - q)/sqrt(1+m^2);
    x_cl  = (m * point(2) - q * m + point(1))/(m^2 + 1);
    x_low = min(segment(1), segment(3));
    x_up  = max(segment(1), segment(3));
    if x_cl > x_up
        x_cl = x_up;
    elseif x_cl < x_low
        x_cl = x_up;
    end
    y_cl = m*x_cl + q;
    p_cl = [x_cl; y_cl];
    d    = norm(p_cl - [point(1); point(2)]);
    
else % in this case the line is vertical line
    %% change reference frame
    R       = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];
    point   = R * point;
    tmp1    = R * [segment(1); segment(2)];
    tmp2    = R * [segment(3); segment(4)];
    segment = [tmp1(1) tmp1(2) tmp2(1) tmp2(2)];
    
    %% then do the same
    
    m = (segment(4) - segment(2))/(segment(3)- segment(1));
    q = segment(2) - m * segment(1);
    % d = abs(point(2) - m*point(1) - q)/sqrt(1+m^2);
    x_cl  = (m * point(2) - q * m + point(1))/(m^2 + 1);
    x_low = min(segment(1), segment(3));
    x_up  = max(segment(1), segment(3));
    if x_cl > x_up
        x_cl = x_up;
    elseif x_cl < x_low
        x_cl = x_up;
    end
    y_cl = m*x_cl + q;
    p_cl = [x_cl; y_cl];
    d    = norm(p_cl - [point(1); point(2)]);
    p_cl = R'*p_cl;
end

