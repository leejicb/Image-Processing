function H = dlt(p1, p2)

% Number of point correspondences
N = size(p1, 2);

%======= IMPLEMENT HERE!! =======%
% Construct the matrix A
A = zeros(2*N, 9);
for i = 1:N
    x = p1(1, i);
    y = p1(2, i);
    u = p2(1, i);
    v = p2(2, i);
    A(2*i-1, :) = [-x, -y, -1, 0, 0, 0, x*u, y*u, u];
    A(2*i, :) = [0, 0, 0, -x, -y, -1, x*v, y*v, v];
end

% Perform Singular Value Decomposition (SVD) of A
[~, ~, V] = svd(A);

% Extract the right singular vector corresponding to the smallest singular value
H = reshape(V(:, end), 3, 3)';
end
%================================%

