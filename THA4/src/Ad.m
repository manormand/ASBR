function AdT = Ad(T)
% Ad returns the Adjoint Representation of a Transformation matrix
%
% Use:
% AdT = Ad(T)
%   - T is a 4x4 Transformation Matrix
arguments
    T (4,4)
end

R = T(1:3,1:3);
p = T(1:3,4);

p_skew = skewify(p);

AdT = [        R  zeros(3);
        p_skew*R        R];