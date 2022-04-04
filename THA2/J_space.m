function J = J_space(S, q)
% J_space computes the space form Jacobian of a screw axis representation
%
% Uses:
% J = J_space(S, q)
%   - S is a 6xn screw axis representation of an _n_ link serial chain
%   - q is a n-element array of the joint positions
%
%   See also J_body

J = S;
T = eye(4);
for j = 2:width(J)
    % Find the adjunct of previous links
    T = T*screwAxis2TMat(S(1:3, j-1), S(4:6, j-1), q(j-1));

    % fill column using Ad and Screw Axis
    J(:,j) = Ad(T)*S(:,j);
end
