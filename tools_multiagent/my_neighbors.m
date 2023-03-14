function neighbors = my_neighbors(x_vec, y_vec, theta_vec, my_index, d_communication)
% The function returns a matrix. The matrix has a column per neighbors.
% Each column contains:
% - x of the neighbor
% - y of the neighbor
% - theta of the neighbor
% - id of the neighbor, i.e., the position in the x_vec
%% Count the number of neighbors
n_neighbors = 0;
for i = 1:length(x_vec)
   if norm([x_vec(i); y_vec(i)]-[x_vec(my_index); y_vec(my_index)]) <= d_communication && i ~= my_index
       n_neighbors = n_neighbors +1;
   end
end
%% Initialize output
neighbors = zeros(4, n_neighbors);
%% Fill output
counter = 1;
for i = 1:length(x_vec)
   if norm([x_vec(i); y_vec(i)]-[x_vec(my_index); y_vec(my_index)]) <= d_communication && i ~= my_index
       neighbors(1,counter) = x_vec(i);
       neighbors(2,counter) = y_vec(i);
       neighbors(3,counter) = theta_vec(i);
       neighbors(4,counter) = i;
       counter              = counter + 1;
   end
end