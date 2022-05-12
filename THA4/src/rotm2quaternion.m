function q = rotm2quaternion(rotm)
% rotm2quaternion Converts a given rotation matrix to a 4x1
% quaternion representation.
%
% Parameters:
%   rotm - 3x3 rotation matrix representation
%
% Returns:
%   q - 4x1 quaternion representation [w,z,y,x]
arguments
    rotm (3,3) double
end

% Setup q and calc scalar val
q = zeros(4,1);
q(1) = 0.5*sqrt( sum(diag(rotm))+1 );

% set indexes for vector calc
idxs = [3 1 2;
        2 3 1];

% solve for vector components
for i=1:3
    a = idxs(1,i);
    b = idxs(2,i);
    q(i+1) = 0.5*sign(rotm(a,b) - rotm(b,a)) * ...
        sqrt( rotm(i,i) - rotm(b,b) - rotm(a,a) + 1 );
end
end