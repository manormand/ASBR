function J = J_body(B, q)
% J_body computes the body form Jacobian of a screw axis representation
%
% Uses:
% J = J_body(B, q)
%   - B is a 6xn screw axis representation of an _n_ link serial chain
%   - q is a n-element array of the joint positions

J = zeros(size(B));
n = width(B);

for j = 1:n
    % Find the adjunct of next links
    T = eye(4);
    for i = j+1:n
        T = T / screwAxis2TMat(B(1:3,i), B(4:6,i), q(i));
    end

    % fill column using Ad and Screw Axis
    J(:,j) = Ad(T)*B(:,j);
end
