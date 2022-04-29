%% PA2 - Animation
% Animates the KUKA LBR IIWA
clear;clc;close all

addpath('src\')

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

q = deg2rad([-30 45 -20 -50 -60 50 0]');

Tsd = FK_body(M,B,q);

%% Inverse Kinematics
close all
% inv_kin_animation(M,B,0.1*ones(size(q)),Tsd)
inv_kin_animation(M,B,zeros(size(q)),Tsd)

%% Transpose Kinematics
close all
trans_kin_animation(M,B,0.1*ones(size(q)),Tsd)

%% Redundancy Resolution Kinematics
close all
% redres_kin_animation(M,B,q_guess,Tsd)
redres_kin_animation(M,B,zeros(size(q)),Tsd)


%%
S = [   0    0 0       0 0            0 0;
        0    1 0      -1 0            1 0;
        1    0 1       0 1            0 1;
        0 -dbs 0 dbs+dse 0 -dbs-dse-dew 0;
        0    0 0       0 0            0 0;
        0    0 0       0 0            0 0];


FK_space(M,S,q,1)

%%
lbr = importrobot('iiwa14.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q);
title('Target Pose', 'FontSize',20)
axis([-0.5 1.0 -0.75 0.75 0, 1.5])