function [point, flag_intersec, flag_parall, flag_visible] = line_segment_intersec2(line, segment)
% line =    [x_l_1 x_l_2 y_l_1 y_l_2], line contains these two points
% segment = [x1 x2 y1 y2]
% flag_intersec = 1 --> the intersection is inside the segment
% flag_intersec = 0 --> the intersection is outside the segment
% flag_parall = 0 they are not parallel
% flag_parall = 1; they are parallel and they do not touch
% flag_parall = 2; they are parallel and they touch (segment contained in
% the line)
% point = [xp;yp], intersection between the two
% flag visible = 1; the intersection is defined, and it is contained in the
% vector started from point [x_l_1 y_l_1] going to [x_l_2 y_l_2]
% flag_visible = -1; the intersection is defined, and it is contained in the
% vector started from point [x_l_1 y_l_1] going in the opposite direction with respect to [x_l_2 y_l_2]

%%
x_1   = segment(1);
y_1   = segment(3);
x_2   = segment(2);
y_2   = segment(4);
x_l_1 = line(1);
y_l_1 = line(3);
x_l_2 = line(2);
y_l_2 = line(4);

toll_vert   = 10^(-2); % tolerance to define if a line is vertical
toll_para   = 10^(-2);
flag_parall = 0;
% dummy initialization
x_intersec    = 0;
y_intersec    = 0;
flag_intersec = 0;
flag_visible  = 0;
%%
if abs(x_1 - x_2) > toll_vert && abs(x_l_1 - x_l_2) < toll_vert
    %% Case 1: only the line is vertical
    x_intersec = (x_l_1 + x_l_2) / 2;
    [m_s, q_s] = from_points2mq(segment); % segment
    y_intersec = m_s * x_intersec + q_s;
elseif abs(x_1 - x_2) < toll_vert && abs(x_l_1 - x_l_2) > toll_vert
    %% Case 2: only the segment is vertical
    x_intersec = (x_1 + x_2) / 2;
    [m_l, q_l] = from_points2mq(line); % line
    y_intersec = m_l * x_intersec + q_l;
elseif abs(x_1 - x_2) < toll_vert && abs(x_l_1 - x_l_2) < toll_vert
    %% Case 3: both are vertical
    if abs((x_1 + x_2) / 2 - (x_l_1 + x_l_2) / 2) < toll_para % the segment is containd in the line
        flag_parall = 2;
    else % the segment is not containd in the line
        flag_parall = 1;
    end
    
else
    %% Case 4: none is vertical
    [m_s, q_s] = from_points2mq(segment); % segment
    [m_l, q_l] = from_points2mq(line); % line
    
    %% Check if they are parallel
    if abs(m_s - m_l) < toll_para
        %% then they are parallel --> undestand if the segment in contained in the line
        if abs(q_s - q_l) < toll_para % the segment is containd in the line since the lines have also the same q
            flag_parall = 2;
        else % the segment is not containd in the line
            flag_parall = 1;
        end
        
    else
        %% Compute intersection if thet are not parallel
        x_intersec = -(q_l - q_s)/(m_l - m_s);
        y_intersec = m_l * x_intersec + q_l;
        
    end
    
end
point = [x_intersec; y_intersec];
%% Now check if the intersection is inside the segment
if flag_parall == 0 % i.e., they are not parallel
    x_max = max(x_1, x_2);
    x_min = min(x_1, x_2);
    y_max = max(y_1, y_2);
    y_min = min(y_1, y_2);
    if (x_intersec - x_max > toll_vert ||  x_intersec < x_min - toll_vert) || (y_intersec - y_max > toll_vert || y_intersec < y_min - toll_vert )
        flag_intersec = 0; % no intersection between segment and line but between lines
    else
        flag_intersec = 1; % intersection
    end
    
end

%% Now handle flag_visible
if flag_parall == 0 %% they are not parallel
    if dot([x_l_2 - x_l_1; y_l_2 - y_l_1], [x_intersec - x_l_1; y_intersec - y_l_1]) > 0
        flag_visible = 1;
    else
        flag_visible = -1;
    end
end

if flag_parall == 2 %% they are parallel and the segment is contained in the line
    if dot([x_l_2 - x_l_1; y_l_2 - y_l_1], [x_1 - x_l_1; y_1 - y_l_1]) > 0
        flag_visible = 1;
    else
        flag_visible = -1;
    end
end

end % function



function [m, q] = from_points2mq(segment)
x_1 = segment(1);
y_1 = segment(3);
x_2 = segment(2);
y_2 = segment(4);
m   = (y_2 - y_1) / (x_2 - x_1);
q   = y_2 - m * x_2;
end