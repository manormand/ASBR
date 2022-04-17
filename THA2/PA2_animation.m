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

q = deg2rad([10 10 1 50 55 20 20]');
%q = 0.5 + (rand([7,1])-0.5)
q_guess = q - (rand([7,1])-0.5);

addpath('src')

Tsd = FK_body(M,B,q);

%% Inverse Kinematics
inv_kin_animation(M,B,ones(7,1),Tsd)

%% Transpose Kinematics
trans_kin_animation(M,B,q_guess,Tsd)

%% Redundancy Resolution Kinematics
redres_kin_animation(M,B,q_guess,Tsd)

%%
close all
trans_pid_kin_animation(M,B,q_guess,Tsd)