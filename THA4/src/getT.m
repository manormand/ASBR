function t = getT(p,M,S,q)
% calc F
F = FK_space(M,S,q);

% calc t
Tp = eye(4); Tp(3,4) = p;

t = F*Tp;
t = t(1:3,4);

