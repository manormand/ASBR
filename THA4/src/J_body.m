function J = J_body(B, q)
% J_body computes the body form Jacobian of a screw axis representation
%
% Uses:
% J = J_body(B, q)
%   - B is a 6xn screw axis representation of an _n_ link serial chain
%   - q is a n-element array of the joint positions
%
%   See also J_space

arguments
    B (6,:)
    q (:,1)
end

J = B;
T = eye(4);
for j = width(J)-1:-1:1
    T = T * screwAxis2TMat(-B(1:3,j+1), -B(4:6,j+1), q(j+1));

    % fill column using Ad and Screw Axis
    J(:,j) = Ad(T)*B(:,j);
end
