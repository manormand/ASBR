function [q_des, itered_out] = J_inverse_kinematics(M, B, q, Tsd, tol_w, tol_v, max_iter)
% J_inverse_kinematics Control robot from config a to b
%   This function uses the Newton Raphson Method to locate
%   the joint positions required for the desired position, Tsd
%
% Use:
% q_des = J_inverse_kinematics(M, B, q0, Tsd)
%   - M is the home position of the robot, represented by a 4x4 T-Mat
%   - B is the body frame screw axes in a 6xn matrix
%   - q is the initial guess configuration in a column vector
%   - Tsd is the desired final configuration
%
% Optional:
% [q_des, itered_out] = J_inverse_kinematics(..., tol_w, tol_v, max_iter)
%   - tol_w is the orientation tolerance (default 0.001)
%   - tol_v is the linear tolerance (default 0.0001)
%   - max_iter is the maximum iterations allowed (default 100)
%   - itered_out shows if the maximum iterations limit was reached
%
%   See also J_transpose_kinematics, redundancy_resolution

arguments
    M (4,4)        % Home postition of end effector
    B (6,:)        % Skew Axes in Body Frame
    q (:,1)        % Initial joint positions
    Tsd (4,4)      % Desired final pose
    tol_w double = 0.001   % orientation tolerance in rad
    tol_v double = 0.0001  % distance tolerance in m
    max_iter int32 = 250     % Maximum iterations allowed
end

% define tolerance function
outside_tolerance = @(Vb) norm(Vb(1:3)) > tol_w || norm(Vb(4:6)) > tol_v;

% get current matrix transforms
Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;

i = 0;
while outside_tolerance(Vb) && i < max_iter
    % iterate to next position
    q = q + J_body(B,q)\Vb;

    % update twist
    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    Vb = S*th;

    i = i+1;
end

% outputs
q_des = wrapToPi(q);

if nargin > 1
    itered_out = i > max_iter;
end