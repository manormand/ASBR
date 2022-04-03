function J = J_space(S, q)
% J_space computes the space form Jacobian of a screw axis representation
%
% Uses:
% J = J_space(S, q)
%   - S is a 6xn screw axis representation of an _n_ link serial chain
%   - q is a n-element array of the joint positions

J = zeros(size(S));
for j = 1:width(S)
    % Find the adjunct of previous links
    T = eye(4);
    for i = 1:j-1
        T = T / screwAxis2TMat(S(1:3,i), S(4:6,i), q(i));
    end

    % fill column using Ad and Screw Axis
    J(:,j) = Ad(T)*S(:,j);
end
