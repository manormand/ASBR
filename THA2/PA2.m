%% HA2.m
% Handwritten Assignment 2 - Supplemental code
%
%%
clear; clc; close all;
addpath("src\") % adds src directory containing functions
% urdf_file = 'urdf/kuka_iiwa7_URDF.xacro';
% robot = getRobotFromURDF(urdf_file);

%% robot params

% link lengths [m]
dbs = 0.340;
dse = 0.400;
dew = 0.400;
dwf = 0.126;

M = eye(4); M(3,4) = dbs+dse+dew+dwf;

q = deg2rad([20 -10 30 0 -30 45 0]);

%% PA.b - Space Form Forward Kinematics
% The function FK_space accepts any serial chain specified in space-form
% screw axes
%
%%
% <include>src/FK_space.m</include>
%
%%
% Kuka LBR example:

% Space form Screw Axes
S = [   0    0 0       0 0            0 0;
        0    1 0      -1 0            1 0;
        1    0 1       0 1            0 1;
        0 -dbs 0 dbs+dse 0 -dbs-dse-dew 0;
        0    0 0       0 0            0 0;
        0    0 0       0 0            0 0];

T_space = FK_space(M,S,q);

fprintf('\tT_space:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_space.')

%% Pa.c - Body Form Forward Kinematics
% The function FK_body accepts any serial chain specified in body-form
% screw axes
%
%%
% <include>src/FK_body.m</include>
%
%%
% Kuka LBR example:

% Body Form Screw Axes
B = [   0           0 0        0 0   0 0;
        0           1 0       -1 0   1 0;
        1           0 1        0 1   0 1;
        0 dse+dew+dwf 0 -dew-dwf 0 dwf 0;
        0           0 0        0 0   0 0;
        0           0 0        0 0   0 0];

T_body = FK_body(M,B,q);

fprintf('\tT_body:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_body.')

%% PA.e - Jacobians
% Space Form
Jacob_space = J_space(S,q);

fprintf('\tJ_space:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_space.')

%%
% Body Form
Jacob_body = J_body(B,q);

fprintf('\tJ_body:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_body.')

%%
Blist = [[0; 0; 1;   0; 0.2; 0.2], ...
       [1; 0; 0;   2;   0;   3], ...
       [0; 1; 0;   0;   2;   1], ...
       [1; 0; 0; 0.2; 0.3; 0.4]];
thetalist = [0.2; 1.1; 0.1; 1.2];
Jb = J_body(Blist, thetalist)
% Output:
% Jb =
%   -0.0453    0.9950         0    1.0000
%    0.7436    0.0930    0.3624         0
%   -0.6671    0.0362   -0.9320         0
%    2.3259    1.6681    0.5641    0.2000
%   -1.4432    2.9456    1.4331    0.3000
%   -2.0664    1.8288   -1.5887    0.4000