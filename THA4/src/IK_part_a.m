function [q_des, q, le] = IK_part_a(M,S,q,J_limits,d_tool,p)
% redundancy_resolution Control robot from config a to b
%   Expansion from J_inverse_kinematics using manipulability as a secondary 
%   objective function. This function calculates manipulability in its own 
%   helper functions and then calculates the gradient using the numerical 
%   derivative formula.
%
% Use:
% q_des = redundancy_resolustion(M, B, q0, Tsd)
%   - M is the home position of the robot, represented by a 4x4 T-Mat
%   - B is the body frame screw axes in a 6xn matrix
%   - q is the initial guess configuration in a column vector
%   - Tsd is the desired final configuration
%
% Optional:
% [q_des, itered_out] = redundancy_resolution(..., tol_w, tol_v, max_iter)
%   - tol_w is the orientation tolerance (default 0.001)
%   - tol_v is the linear tolerance (default 0.0001)
%   - max_iter is the maximum iterations allowed (default 100)
%   - itered_out shows if the maximum iterations limit was reached
%
%   See also J_inverse_kinematics, J_transpose_kinematics

% arguments
%     M (4,4)        % Home postition of end effector
%     B (6,:)        % Skew Axes in Body Frame
%     q (:,1)        % Initial joint positions
%     Tsd (4,4)      % Desired final pose
%     tol_w double = 0.001   % orientation tolerance in rad
%     tol_v double = 0.0001  % distance tolerance in m
%     max_iter int32 = 250     % Maximum iterations allowed
% end

tol_v = 0.003;
max_iter = 400;
h = 1;

R = axangle2rotm([0 1 0], pi);
Tsd = [ R p; [0 0 0 1]];

% define tolerance function
outside_tolerance = @(D) norm(D) > tol_v;

% init loop vals
t = getT([0,0,d_tool]',M,S,q);
D = Tsd\[t;1]; D = D(1:3);
le = norm(D);
i = 1;

while outside_tolerance(D) && i < max_iter
    t = getT([0,0,d_tool]',M,S,q(:,i));

    q(:,i+1) = q(:,i) + calcDq(S,q(:,i),t,p)*h;
    q(:,i+1) = jointLimiter(q(:,i+1),J_limits);

    % update twist
    D = Tsd\[t;1]; D = D(1:3);
    le(i+1) = norm(D);
    i = i+1;
end

q_des = q(:,end);
end

function dq = calcDq(S,q,t,p)
J = J_space(S,q);
J_a = J(1:3,:);
J_e = J(4:6,:);

A = -skewify(t)*J_a + J_e;
b = p - t;
dq = A\b;
end

function q = jointLimiter(q,J_limits)
upper = J_limits(:,2);
lower = J_limits(:,1);
q = min(q,upper);
q = max(q,lower);
end