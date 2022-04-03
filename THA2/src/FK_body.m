function T = FK_body(M,B,q)
% FK_body returns the body-form forward kinematics of the robot
% manipulator, defined by the home position M and the screw axes B.
%
% Use:
% T = FK_body(M,B,q)
%   - M is the home position Transform
%   - B is a 6xn matrix of the Body-Form Screw Axes for each joint
%   - q is an n-element array of joint positions
T = M;
for i=1:width(B)
    T_i = screwAxis2TMat(B(1:3,i), B(4:6,i), q(i));
    T = T*T_i;
end
end
