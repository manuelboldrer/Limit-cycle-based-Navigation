function center_matrix = metropolis_mean(x_vec, y_vec, communication_steps, d_communication)
% center_matrix is a matrix having n_agents = length(x_vec) rows and 2
% columns. The j-th row contains the x and the y of the estimated center
% by agent j after a number of communication steps equal to
% communication_steps
%% Initialize center estimate
center_matrix = zeros(length(x_vec), 2);
for j = 1:length(x_vec)
    center_matrix(j,:) = [x_vec(j), y_vec(j)];
end
%% Compute the neighbors for each agent
neighbor_id = cell(length(x_vec),1);
for j = 1:length(x_vec)
    neighbors      = my_neighbors(x_vec, y_vec, x_vec*0, j, d_communication);
    neighbor_id{j} = neighbors(4,:);
end
%% Initialize matrix A of the weights
A = zeros(length(x_vec));
% Fill off diagonal terms
for i = 1:length(x_vec)
    for j = 1:length(x_vec)
        if i~=j && a_in_vector(i, neighbor_id{j}) == 1
            A(i,j) = 1/(1 + max(length(neighbor_id{i}),length(neighbor_id{j})));
        end
    end
end
% Fill diagonal terms
for i = 1:length(x_vec)
    A(i,i) = 1 - sum(A(i,:));
end

%% Comunicate to improve your estimate
for i = 1:communication_steps
    center_matrix(:,1) = A * center_matrix(:,1);
    center_matrix(:,2) = A * center_matrix(:,2);
end

end

function flag = a_in_vector(a,v)
% retur 1 if a is inside vector v, 0 otherwise
flag = 0;
for i = 1:length(v)
    if a == v(i)
        flag = 1;
    end
end
end





