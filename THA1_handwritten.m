%% ASBR THA1_handwritten.m
% Useful operations used in the handwritten portion of THA1
%%
clear; clc

%% Handwritten 3a
syms w1 w2 w3 th lamb
w = [w1 w2 w3].';

w_skew = Rotation.skewify(w);

disp('---w_skew Matrix Reduction with Eigenvalues---')
for lambda = [0 1i -1i]
    mat = w_skew + lambda*eye(3);
    mat_reduced = reduceMatrix(mat);

    fprintf('lambda = %d%+dj\n',real(lambda),imag(lambda))
    disp(mat_reduced)
end


%% Handwritten 7d
Tas = [  0  0 1 0;
        -1  0 0 3;
         0 -1 0 0;
         0  0 0 1];
Tsb = [ 1  0 0 0;
        0  0 1 2;
        0 -1 0 0;
        0  0 0 1];
Tab = Tas*Tsb;

disp('Tab:')
disp(Tab)

%% Handwritten 7e
Tsa = [ 0 -1  0 3;
        0  0 -1 0;
        1  0  0 0;
        0  0  0 1];
T1 = Tsa*Tsb;
T2 = Tsb*Tsa;

disp('T1:')
disp(T1)
disp('T2:')
disp(T2)
%% Handwritten 7f
pb = [1 2 3]';

pbs = Tsb*[pb;1];
pbs = pbs(1:3);

disp('pbs:')
disp(pbs)

%% Handwritten 7g
ps = [1 2 3 1]';
p_prime = Tsb*ps;
p_pprime = Tsb\ps;

disp('p_prime:')
disp(p_prime)
disp('pp_prime:')
disp(pp_prime)

%% Handwritten 7h
twist_s = [3 2 1 -1 -2 -3]';
Rsa = Tsa(1:3,1:3);
psa = Tsa(1:3,4);
psa_skew = [      0  -psa(3)  psa(2);
             psa(3)        0 -psa(1);
            -psa(2)   psa(1)      0];

twist_a = [            Rsa' zeros(3);
            -Rsa'*psa_skew      Rsa'] * twist_s;

disp('twist_a:')
disp(twist_a)

%% Handwritten 7j
clc; close all
load("ex.mat")
% defined:
% p, R, th, v, w
T = [ R p; 0 0 0 1];

b = tan(th/2)*w;
d = p;
d_star = d - b;

q = cross(b, (d_star - cross(b,d_star)) ) / (2*dot(b,b));

screw_ax_length = 10;
screw_ax = [(q-w*screw_ax_length) q (q+w*screw_ax_length)];

figure()
plot3(screw_ax(1,:), screw_ax(2,:), screw_ax(3,:), 'm:', LineWidth=3)
    hold on
plot3(0,0,0,'ko')
plot3(p(1),p(2),p(3), 'ro')
plot3([0 1],[0 0],[0 0], 'b')
plot3([0 0],[0 1],[0 0], 'r')
plot3([0 0],[0 0],[0 1], 'g')
    title('Problem 7j')
    xlabel('X'), ylabel('Y'), zlabel('Z'),
    axis equal
    grid on

col = ["b" "r" "g"];
axes = eye(4,3); axes(end,:) = 1;
for i = 1:3
    ax = T*axes(:,i);
    X = [p(1) ax(1)];
    Y = [p(2) ax(2)];
    Z = [p(3) ax(3)];

    plot3(X,Y,Z,col(i))
end

legend(["$S_1$","$\{s\} O$","$\{b\} O$"],'Interpreter','latex', 'FontSize',14)
xlim(xl), ylim(yl), zlim(zl)