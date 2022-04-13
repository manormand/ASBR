%% PA2 - Animation
% Animates the KUKA LBR IIWA
clear;clc;close all

%% Init Robot Params
dbs = 0.340;
dse = 0.400;
dew = 0.400;
dwf = 0.126;

% home position
M = eye(4); M(3,4) = dbs+dse+dew+dwf;


B = [   0           0 0        0 0   0 0;
        0           1 0       -1 0   1 0;
        1           0 1        0 1   0 1;
        0 dse+dew+dwf 0 -dew-dwf 0 dwf 0;
        0           0 0        0 0   0 0;
        0           0 0        0 0   0 0];

% q = deg2rad([30 50 30 50 -30 60 0]');
q = rand([7,1])

addpath('src')

%% Inverse Kinematics
Tsd = FK_body(M,B,q);

inv_kin_animation(M,B,ones(7,1),Tsd)

%% Transpose Kinematics