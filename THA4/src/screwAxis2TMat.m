function T_mat = screwAxis2TMat(w,v,theta)
% screwAxis2TMat converts Screw Axes to their equivalent Transformation
% Matrix representation
%
% Use:
% T_mat = screwAxis2TMat(w,v,theta)
%   - w and v are the 3x1 properties of the screw axis
%   - theta is the distance traveled on the screw axis
%   - T_mat is the 4x4 Transformation Matrix
%
%   See also tMat2ScrewAxis
arguments
    w (3,1)
    v (3,1)
    theta
end

% calculate rotation
w_skew = skewify(w);
R = eye(3) + sin(theta)*w_skew + (1-cos(theta))*w_skew^2;

% Calculate translation
p = (eye(3)*theta + (1-cos(theta))*w_skew + (theta-sin(theta))*w_skew^2)*v;

T_mat = [         R  p;
          zeros(1,3) 1];
end