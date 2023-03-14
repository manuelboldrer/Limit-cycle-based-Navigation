function [x, y] = id2xy(i, laserData_struc)
    r     = laserData_struc.ranges(i);
    alpha = laserData_struc.range_min + (i-1)*laserData_struc.angle_increment;
    x     = r * cos(alpha);
    y     = r * sin(alpha);
end