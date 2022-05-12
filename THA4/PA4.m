%% PA 4 - w/ Kuka LBR
%
% Authors: Pranav Kalyani,
%
%          Michael Normand
%
% Date: May 12, 2022
%
clear; clc; close all

addpath('src')

%% Robot Params
% These were defined from the paper TODO: Insert paper
% d* is the z-length between joint segments, where joints are grouped into
% couples of z-axis and y-axis revolute joints
%
% * dbs is between base and j2/j3 (shoulder)
% * dse is between j2/j3 and j4/j5 (elbow)
% * dew is between j4/j5 and j6/j7 (wrist)
% * dwf is between j6/j7 and end effector (finger)
%

% link lengths [m]
dbs = 0.340;
dse = 0.400;
dew = 0.400;
dwf = 0.126;

d_tool = 0.100;

% home position
M = eye(4); M(3,4) = dbs+dse+dew+dwf;

J_limits = deg2rad([-170 -120 -170 -120 -170 -120 -175;
                     170  120  170  120  170  120  175]');


% Space form Screw Axes
S = [   0    0 0       0 0            0 0
        0    1 0      -1 0            1 0
        1    0 1       0 1            0 1
        0 -dbs 0 dbs+dse 0 -dbs-dse-dew 0
        0    0 0       0 0            0 0
        0    0 0       0 0            0 0];

% arbitrary joint positions
q0 = 0.1*ones(7,1);

%% PA.a - Space Form Forward Kinematics
clc; close all
p_goal = [0.5 0.5 0.5]';

[q_des,q, le] = IK_part_a(M,S,q0,J_limits,d_tool,p_goal);

rad2deg(q_des)

figure()
plot(le)
    ylim([0 1.2*max(le)])

figure()
lbr = importrobot('iiwa14.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
gripper = 'iiwa_link_ee_kuka';
show(lbr,q_des);
xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('KUKA LBR iiwa 14')

