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