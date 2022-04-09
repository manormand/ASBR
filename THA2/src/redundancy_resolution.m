function [q_des, itered_out] = redundancy_resolution(M,B,q,Tsd, tol_w, tol_v, max_iter) 
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

% init loop vals
k0 = 0.1;   % must be > 0
i = 0;

% get current matrix transforms
Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;

while outside_tolerance(Vb) && i < max_iter
    q = q + J_body(B,q)\Vb + k0*manipulability_gradient(B,q);
    
    % update twist
    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    Vb = S*th;

    i = i+1;
end

q_des = wrapToPi(q);

if nargin > 1
    itered_out = i > max_iter;
end
end


%%%%%%%%%%%%%%%%%%%%%%% Manipulatbility funcs %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function m = manipulability(B,q)
% manipulability calculates the manipulability measure at a certain state.
%   If the manipulabiility is undefined, return 0

% get inner val before sqrt
a = det(J_body(B,q)*J_body(B,q)');

if a > 0
    m = sqrt(a);
else
    m = 0;
end
end

function m_grad = manipulability_gradient(B, q)
% Calculates the manipulability gradient at point q
m = @(q) manipulability(B,q);

dq = 0.001;
n = length(q);
m_grad = zeros(n,1);

for i = 1:n
    E = zeros(n,1);
    E(i) = 1;

    m_grad(i) = ( m(q + dq*E) - m(q - dq*E) )...
                    /(2*dq);
end
end