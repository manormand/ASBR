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

% Space form Screw Axes
S = [   0    0 0       0 0            0 0;
        0    1 0      -1 0            1 0;
        1    0 1       0 1            0 1;
        0 -dbs 0 dbs+dse 0 -dbs-dse-dew 0;
        0    0 0       0 0            0 0;
        0    0 0       0 0            0 0];

%% PA.b
% We use the following function to compute the forward kinematics of any
% serial chain manipulator
%
%%
% <include>src/FK_space.m</include>
%
%%
% Use:
q = deg2rad([0 0 0 0 0 0 0]);
T_space = FK_space(M,S,q);

fprintf('\tT_space:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_space.')

%% Pa.c
