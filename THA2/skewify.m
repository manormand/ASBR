function w_skew = skewify(w)
% skewify returns the skew symmetric matrix representation of a vector
%
% Uses:
% w_skew = skewify(w)
%   - w is a 3 element array
%   - w_skew is a 3x3 skew symmetric matrix
w_skew = [     0 -w(3)  w(2);
            w(3)     0 -w(1);
           -w(2)  w(1)    0];