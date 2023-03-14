function [R, U_0] = tune_potential(d_c, d_inf, F_c, F_inf)
% d_c   = distance at the collision
% F_c   = Forceat d_c
% d_inf = infinite distance
% F_inf = Force an d_inf
R   = (d_c - d_inf)/log(F_inf/F_c);
U_0 = F_c * R * exp(d_c/R);
