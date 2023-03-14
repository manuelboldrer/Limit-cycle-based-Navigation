function plot_obj = plot_cone(x, y, h_reference, angle_big, angle_small)
h_reference = 2*h_reference;

[P1, P2, P3] = generate_points(x, y, angle_big, h_reference);
plot1        = plot([P1(1) P2(1)],[P1(2) P2(2)], 'color', 'k', 'linestyle','--', 'linewidth', 1);
plot2        = plot([P1(1) P3(1)],[P1(2) P3(2)], 'color', 'k', 'linestyle','--', 'linewidth', 1);

[P1, P2, P3] = generate_points(x, y, angle_small, h_reference);
plot3        = plot([P1(1) P2(1)],[P1(2) P2(2)], 'color', 'k', 'linestyle','--', 'linewidth', 1);
plot4        = plot([P1(1) P3(1)],[P1(2) P3(2)], 'color', 'k', 'linestyle','--', 'linewidth', 1);

plot_obj = [plot1, plot2, plot3, plot4];

end

function [P1, P2, P3] = generate_points(x, y, angle, h_reference)
%% define points of the cone in walker frame
P1 = [0;0];
P2 = rot_matrix(angle) * h_reference;
P3 = rot_matrix(-angle) * h_reference;
%% Put in absolute frame
P1 = P1 + [x;y];
P2 = P2 + [x;y];
P3 = P3 + [x;y];
end

function R = rot_matrix(angle)
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
end