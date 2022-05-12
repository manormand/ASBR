function t = getT(p,M,S,q)
% calc F
F = FK_space(M,S,q);

% calc t
p = [p; 1];

t = F*p;
t = t(1:3);

