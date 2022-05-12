function [axis,angle] = rotm2axangle(rotm)
% rotm2axangle Converts a given rotation matrix to
% axis angle representation.
%
% Parameters:
%   rotm - 3x3 rotation matrix representation
%
% Returns:
%   axis - axis of rotation as a 3x1 vector
%   angle - angle of rotation in radians
%
%   See also skewify
arguments
    rotm (3,3) double
end

tr_R = sum(diag(rotm)); % Trace of rotm, tr(R)
tolerance = 0.001;

if rotm==eye(3)
    % Case 1: Angle = 0
    angle = 0;
    axis = [1; 0; 0]; % axis is undefined. Default to x-axis.
elseif abs(tr_R+1) <= tolerance
    % Case 2: Angle = pi
    angle = pi;

    % Make sure the denominator is nonzero
    if rotm(1,1) ~= -1
        axis = (2*(1+rotm(1,1)))^-0.5 * (rotm(:,1) + [1;0;0]);
    elseif rotm(2,2) ~= -1
        axis = (2*(1+rotm(2,2)))^-0.5 * (rotm(:,2) + [0;1;0]);
    elseif rotm(3,3) ~= -1
        axis = (2*(1+rotm(3,3)))^-0.5 * (rotm(:,3) + [0;0;1]);
    end
else
    % Case 3: General case
    angle = acos( 0.5*(tr_R - 1) );
    w_skew = ( 1/(2*sin(angle)) ) * (rotm - rotm');
    axis = [w_skew(3,2); w_skew(1,3); w_skew(2,1)];
end
end