function T = FK_space(M,S,q)
% FK_space returns the space-form forward kinematics of the robot
% manipulator, defined by the home position M and the screw axes S.
%
% Use:
% T = FK_space(M,S,q)
%   - M is the home position Transform
%   - S is a 6xn matrix of the Screw Axes for each joint
%   - q is an n-element array of joint positions
T = eye(4);
for i=1:width(S)
    T_i = screwAxis2TMat(S(1:3,i), S(4:6,i), q(i));
    T = T*T_i;
end
T = T*M;
end
