function [flag_v, flag_omega, h_ref] = control_authority(theta, params, F_total, F_permanent, hysteresis_flag)
%% it returns two flags stating who controls v and omega
% flag_v     = 0 the user controls v
% flag_omega = 0 the user controls omega
% flag_v     = 1 the robot controls v
% flag_omega = 1 the robot controls omega
% h_ref is the heading to stabilize when the robot is in control
%% Read parameters

angle_cone_big   = params.safe_angle_big;
angle_cone_small = params.safe_angle_small;   

%% Evaluate the working conditions (i.e., far from local minima or not)
h               = [cos(theta); sin(theta)]; % vehicle heading
h_f             = F_total/norm(F_total); % heading of the total force. This is not defined in local minima%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
local_mimina_ok = true;
% compute the angle between permnent force and total force and the norm of the total force
h_permanent = F_permanent/norm(F_permanent); % heading of the permanent force, not defined in some socal minima%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if dot(h_permanent, h_f) <= cos(90*pi/180) || norm(F_total) < 0.1
%     local_mimina_ok = false;
% end
%% Decides who has control authority
if local_mimina_ok  
    scalar_prod = dot(h, h_f);
    if (scalar_prod <= cos(angle_cone_big) && hysteresis_flag == 0) || scalar_prod >= cos(angle_cone_small) && hysteresis_flag == 1 % change the control authority
        flag_v     = 1 - hysteresis_flag;
        flag_omega = 1 - hysteresis_flag;
    else
        flag_v     = hysteresis_flag;
        flag_omega = hysteresis_flag;
    end
    
    %%
    h_ref = h_f;
else % we have problems of local minima
     %% Robot always in control on v
     flag_v = 1;
    %% decides control authority on omega
    scalar_prod = dot(h, h_permanent);
    if (scalar_prod <= cos(angle_cone_big)   && hysteresis_flag == 0) || scalar_prod >= cos(angle_cone_small) && hysteresis_flag == 1
        flag_omega = 1 - hysteresis_flag;
    else
        flag_omega = hysteresis_flag;
    end
    %%
    h_ref = h_permanent;
end
% In pracetice: if there are no problems with local minima, either the
% robot is in control, or the user is in control. If there are problems
% with local minima, the vehicle brakes and the attitude of the permanent
% field is stabilized