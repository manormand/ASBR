%% ASBR HW1 Programming Assignment
% In this assignment we extensively use the class Rotation, defined in a
% separate file.

%% General + Scratch
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

%% Problem 1
% define givens
theta = deg2rad(30);
w = [0; 0.866; 0.5];

% Get Euler Angles
rotm = Rotation.axangle2rotm(w,theta);
Phi = Rotation.rotm2euler(rotm, 'RPY');

% check validity
Phi_act = [0.0644047, 0.4478289, 0.2810408]';
tolerance = 0.001;
rotm_check = abs(Phi-Phi_act) <= tolerance
