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
q0 = ones(7,1);
p_goal = [0.5 0.5 0.5]';

% plot initial robot position
figure(1)
lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q0, 'Frames', 'off');
hold on;
T0 = FK_space(M,S,q0);
p_tool = [0 0 d_tool 1]';
p_end = T0*p_tool;

X = [T0(1,4) p_end(1)];
Y = [T0(2,4) p_end(2)];
Z = [T0(3,4) p_end(3)];

plot3(X,Y,Z,'g', 'LineWidth',5)
plot3(p_goal(1),p_goal(2),p_goal(3), 'ro','MarkerFaceColor','r')
hold off

xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('Original Robot Position')

%% PA.a - Space Form Forward Kinematics
[q_des_a,~, le] = IK_part_a(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_a)

% plot linear error
figure(2)
plot(le)
    ylim([0 1.2*max(le)])

% plot final robot position
figure(3)
lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q_des_a, 'Frames', 'off');
hold on;
T1 = FK_space(M,S,q_des_a);
p_tool = [0 0 d_tool 1]';
p_end = T1*p_tool;

X = [T1(1,4) p_end(1)];
Y = [T1(2,4) p_end(2)];
Z = [T1(3,4) p_end(3)];

plot3(X,Y,Z,'g', 'LineWidth',5)
plot3(p_goal(1),p_goal(2),p_goal(3), 'ro','MarkerFaceColor','r')
hold off

xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('Final Robot Position w/ Joint Limits')


%% PA.b
[q_des_b,~, le] = IK_part_b(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_a)

% plot linear error
figure(2); hold on;
plot(le)
    ylim([0 1.2*max(le)])

% plot final robot position
figure(4)
lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q_des_b, 'Frames', 'off');
hold on;
T1 = FK_space(M,S,q_des_b);
p_tool = [0 0 d_tool 1]';
p_end = T1*p_tool;

X = [T1(1,4) p_end(1)];
Y = [T1(2,4) p_end(2)];
Z = [T1(3,4) p_end(3)];

plot3(X,Y,Z,'g', 'LineWidth',5)
plot3(p_goal(1),p_goal(2),p_goal(3), 'ro','MarkerFaceColor','r')
hold off

xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('Final Robot Position w Joint Restraints + orientation control')

%% PA.c
