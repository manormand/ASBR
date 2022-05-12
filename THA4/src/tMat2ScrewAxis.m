function [S, th] = tMat2ScrewAxis(T)
% tMat2ScrewAxis Convert Transformation Matrix to Screw axis representation
%
% Use:
% [S, th] = tMat2ScrewAxis(T)
%   - T is the 4x4 Transformation Matrix
%   - S is 6x1 representation of the Screw Axis
%   - th is the distance along the screw axis
%
%   See also screwAxis2Tmat, rotm2axangle, skewify

arguments
    T (4,4)
end

R = T(1:3, 1:3);
p = T(1:3, 4);

% Special case: no rotiation
if R==eye(3)
    w = [0 0 0]';
    th = norm(p);
    v = p/th;

    S = [w; v];
    return
end

[w, th] = rotm2axangle(R);
w_skew = skewify(w);

G_inv = (1/th)*eye(3) - 0.5*w_skew + ((1/th) - 0.5*cot(th/2))*w_skew^2;
v = G_inv*p;

S = [w; v];
end