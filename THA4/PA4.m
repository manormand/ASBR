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

%% Initial conditions
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
[q_des_a,~, le_a] = IK_part_a(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_a)

% plot linear error
figure(2)
plot(le_a)
    ylim([0 1.2*max(le_a)])

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
[q_des_b,~, le_a] = IK_part_b(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_a)

% plot linear error
figure(2); hold on;
plot(le_a)
    ylim([0 1.2*max(le_a)])

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
title('Joint Restraints + orientation control')

%% PA.c - a
n = [2 -1 2]';
[q_des_a,~, le_a] = IK_part_cA(M,S,q0,J_limits,d_tool,p_goal,n);
final_q = rad2deg(q_des_a)

figure(2)
plot(le_a)
    ylim([0 1.2*max(le_a)])

figure(5)
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
patch([n(1) 0 0], [0 n(2) 0], [0 0 n(3)], [1 1 1])
hold off
xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('Joint Restraints w/ virtual wall');

%% PA.c - b
n = [2 -1 2]';
[q_des_a,~, le_a] = IK_part_cA(M,S,q0,J_limits,d_tool,p_goal,n);
final_q = rad2deg(q_des_a)

figure(2)
plot(le_a)
    ylim([0 1.2*max(le_a)])

figure(6)
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
patch([n(1) 0 0], [0 n(2) 0], [0 0 n(3)], [1 1 1])
hold off
xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
title('Joint Restraints + orientation control w/ virtual wall');

%% Part d - Configs

% init configurations
n1 = n;
n2 = -n;
n3 = [1 0.5 0.5]';

p1 = p_goal;
p2 = [-0.5 0.5 0.5]';
p3 = [0 0.6 0.2]';

%%
% Testing for part a
[q_des, ~, le1] = IK_part_a(M,S,q0,J_limits,d_tool,p1);
[q_des, ~, le2] = IK_part_a(M,S,q_des,J_limits,d_tool,p2);
[~, ~, le3] = IK_part_a(M,S,q_des,J_limits,d_tool,p3);

figure()
le_a = [le1 le2 le3];
plot(le_a), hold on;
xline([length(le1) length(le1)+length(le2)], ':');
    hold off;
    title('Linear Error: Joint Restraints')
    ylim([0 1.2*max(le_a)])

%%
% Testing for part b
[q_des, ~, le1] = IK_part_b(M,S,q0,J_limits,d_tool,p1);
[q_des, ~, le2] = IK_part_b(M,S,q_des,J_limits,d_tool,p2);
[~, ~, le3] = IK_part_b(M,S,q_des,J_limits,d_tool,p3);

figure()
le_b = [le1 le2 le3];
plot(le_b), hold on;
xline([length(le1) length(le1)+length(le2)], ':');
    hold off;
    title('Linear Error: Joint Restraints + orientation lock')
    ylim([0 1.2*max(le_b)])

%%
% Testing for part c-a
[q_des, ~, le1] = IK_part_cA(M,S,q0,J_limits,d_tool,p1,n1);
[q_des, ~, le2] = IK_part_cA(M,S,q_des,J_limits,d_tool,p2,n2);
[~, ~, le3] = IK_part_cA(M,S,q_des,J_limits,d_tool,p3,n3);

figure()
le_cA = [le1 le2 le3];
plot(le_cA), hold on;
xline([length(le1) length(le1)+length(le2)], ':');
    hold off;
    title('Linear Error: Joint Restraints + Virtual Wall')
    ylim([0 1.2*max(le_cA)])

%%
% Testing for part c-b
[q_des, ~, le1] = IK_part_cB(M,S,q0,J_limits,d_tool,p1,n1);
[q_des, ~, le2] = IK_part_cB(M,S,q_des,J_limits,d_tool,p2,n2);
[~, ~, le3] = IK_part_cB(M,S,q_des,J_limits,d_tool,p3,n3);

figure()
le_cB = [le1 le2 le3];
plot(le_cB), hold on;
xline([length(le1) length(le1)+length(le2)], ':');
    hold off;
    title('Linear Error: All Restraints')
    ylim([0 1.2*max(le_cB)])

%% final plot?
figure()
plot(le_a); hold on
plot(le_b)
plot(le_cA)
plot(le_cB)
    title('Linear Error Comparison')
    ylabel('error [m]')
    xlabel('iteration')
    legend('Part a','Part b','Part c-a','partc-b')
