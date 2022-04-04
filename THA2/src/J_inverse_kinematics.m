function q_des = J_inverse_kinematics(M, B, q, Tsd, tol_w, tol_v)
% J_inverse_kinematics Control robot from config a to b
%   This function uses the numerical inverse kinematics algorithm to
%   control the robot from config a to b
%
% Use:
% q_des = J_inverse_kinematics(M, B, q0, Tsd)
%   - M is the home position of the robot, represented by a 4x4 T-Mat
%   - B is the body frame screw axes in a 6xn matrix
%   - q is the initial guess configuration in a column vector
%   - Tsd is the desired final configuration
%
% q_des = J_inverse_kinematics(M, ..., tol_w, tol_v)
%   - tol_w is the orientation tolerance
%   - tol_v is the linear tolerance
%
%   See also Fk_body, tMat2ScrewAxis, J_body

arguments
    M (4,4)        % Home postition of end effector
    B (6,:)        % Skew Axes in Body Frame
    q (1,:)        % Initial joint positions
    Tsd (4,4)      % Desired final pose
    tol_w = 0.001  % orientation tolerance in rad
    tol_v = 0.0001 % distance tolerance in m
end

% define tolerance function
outside_tolerance = @(Vb) norm(Vb(1:3)) > tol_w || norm(Vb(4:6)) > tol_v;

% get current matrix transforms
Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;

while outside_tolerance(Vb)
    % iterate to next position
    q = q + pinv( J_body(B,q) )*Vb;

    % update twist
    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    Vb = S*th;
end

q_des = q;
