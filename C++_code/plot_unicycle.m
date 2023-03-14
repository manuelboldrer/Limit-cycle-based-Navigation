function output = plot_unicycle(x, y, theta, color, dimension)
%% Parameter
scale        = dimension;
L            = 0.35*scale; % vehicle length
W            = 0.4*scale; % vehicle width
linewidth    = 1; %width of the plot line
arrow_length = 0.1*scale;
% color        = 'k';
wheel_length = 0.3*scale;
%% Plot central arrow
R              = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % rotation matrix
arrow_position = [x; y] + R * [L; 0];
hold on;
daspect([1 1 1]);
plot1 = plot([arrow_position(1) x], [arrow_position(2) y], 'linewidth', linewidth, 'color', color);

angle = 90 + 45 ;
tmp = arrow_position + [cos(theta + angle*pi/180), -sin(theta + angle*pi/180); sin(theta + angle*pi/180), cos(theta + angle*pi/180)] * [arrow_length; 0];
plot2 = plot([arrow_position(1) tmp(1)], [arrow_position(2) tmp(2)], 'linewidth', linewidth, 'color', color);

angle = -angle;
tmp = arrow_position + [cos(theta + angle*pi/180), -sin(theta + angle*pi/180); sin(theta + angle*pi/180), cos(theta + angle*pi/180)] * [arrow_length; 0];
plot3 = plot([arrow_position(1) tmp(1)], [arrow_position(2) tmp(2)], 'linewidth', linewidth, 'color', color);

%% Plot wheel and rear axle
angle       = 90;
R           = [cos(theta + angle*pi/180), -sin(theta + angle*pi/180); sin(theta + angle*pi/180), cos(theta + angle*pi/180)]; % rotation matrix
wheel_left  = [x; y] + R * [W/2; 0];
wheel_right = [x; y] + R * [-W/2; 0];
plot4 = plot([wheel_left(1) wheel_right(1)], [wheel_left(2) wheel_right(2)], 'linewidth', linewidth, 'color', color);

wheel             = wheel_left;
R                 = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % rotation matrix
wheel_front_point = wheel + R * [wheel_length/2; 0];
wheel_rear_point  = wheel + R * [-wheel_length/2; 0];
plot5 = plot([wheel_front_point(1) wheel_rear_point(1)], [wheel_front_point(2) wheel_rear_point(2)], 'linewidth', linewidth, 'color', color);

wheel             = wheel_right;
wheel_front_point = wheel + R * [wheel_length/2; 0];
wheel_rear_point  = wheel + R * [-wheel_length/2; 0];
plot6 = plot([wheel_front_point(1) wheel_rear_point(1)], [wheel_front_point(2) wheel_rear_point(2)], 'linewidth', linewidth, 'color', color);

plot7 = circle(x+R(1,:)*[0;0],y+R(2,:)*[0.;0],.0,DarkOrange);
% plot8 = scatter(x+R(1,:)*[0;0],y+R(2,:)*[0.2;0],'ro','filled');
output = [plot1, plot2, plot3, plot4, plot5, plot6, plot7];