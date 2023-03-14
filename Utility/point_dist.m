function dist = point_dist(id1, id2,laserData_struc)
[x1, y1] = id2xy(id1, laserData_struc);
[x2, y2] = id2xy(id2, laserData_struc);
dist     = sqrt((x1-x2)^2 + (y1-y2)^2);
