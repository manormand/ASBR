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

%%
% arbitrary joint positions and goals
q0 = ones(7,1);
p_goal = [0.5 0.5 0.5]';

%% Display Initial Conditions
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

%%

%% PA.a
% Here we perform inverse kinematics with joint limitation implemented.
% Depicted in the figure is the final robot position after it is deemed
% close enough.
%
[q_des_a,~, le_a,cond_ang,cond_lin,iso_ang,iso_lin] = IK_part_a(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_a)

% plot linear error
figure(2)
plot(le_a)
    ylim([0 1.2*max(le_a)])
    title('Linear Error')
    ylabel('error [m]')
    xlabel('iteration')
    

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

figure(4)
plot(cond_ang)
hold on 
plot(cond_lin)
title('Condition number a')
hold off
legend('Cond angular a','Cond linear a')

figure(5)
plot(iso_ang)
hold on 
plot(iso_lin)
title('Isotropy number a')
hold off 
legend('Iso angular a','Iso linear a ')


%%
% Discusion - These results are actually quite nice. Only 7 iterations? I'll take it. It
% should be noted that the tool was not oriented properly. Heck the robot
% is a little funky. But hey, it works!

%% PA.b
% Now we add a littl bit of *Pazzaz*. We can restrict the orientation to be
% pointed directly down with a weighted goal.
[q_des_b,~, le_b, cond_ang_b,cond_lin_b,iso_ang_b,iso_lin_b] = IK_part_b(M,S,q0,J_limits,d_tool,p_goal);

final_q = rad2deg(q_des_b)

% plot linear error
figure(6); hold on;
plot(le_b)
    ylim([0 1.2*max(le_b)])
    title('Linear Error')
    ylabel('error [m]')
    xlabel('iteration')

% plot final robot position
figure(7)
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
    
 figure(8)
plot(cond_ang_b)
hold on 
plot(cond_lin_b)
title('Condition number b')
hold off
legend('Cond angular b','Cond linear b')

figure(9)
plot(iso_ang_b)
hold on 
plot(iso_lin_b)
title('Isotropy number b')
hold off 
legend('Iso angular b','Iso linear b')
    

%%
% Discusion - The iteration count nearly tripled to 20 iterations, yet the orientation
% was preserved despite starting at the oposite orientation (tool was
% upward). Much more confident with this one.

%% PA.c - a
% Now we add a pesky wall. I deefined it in 3 points, shown in the robot
% figure.
n = [2 -1 2]';
[q_des_cA,~, le_cA,cond_ang_Ca,cond_lin_Ca,iso_ang_Ca,iso_lin_Ca] = IK_part_cA(M,S,q0,J_limits,d_tool,p_goal,n);
final_q = rad2deg(q_des_cA)

figure(10)
plot(le_cA)
    ylim([0 1.2*max(le_cA)])
    title('Linear Error')
    ylabel('error [m]')
    xlabel('iteration')

figure(11)
lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q_des_cA, 'Frames', 'off');
hold on;
T1 = FK_space(M,S,q_des_cA);
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

figure(12)
plot(cond_ang_Ca)
hold on 
plot(cond_lin_Ca)
title('Condition number Ca')
hold off
legend('Cond angular Ca','Cond linear Ca')

figure(13)
plot(iso_ang_Ca)
hold on 
plot(iso_lin_Ca)
title('Isotropy number Ca')
hold off 
legend('Iso angular Ca','Iso linear Ca')



%%
% Discusion - Man that wall came out of no where. Because I did not add any more
% compensation, I expected the wall to mess with the simulation. The robot
% does not even converge, which kinda sucks. You can see that the final q
% hits the joint limits of the robot, most likely why we see this
% divergence.

%% PA.c - b
% Now we add every restraint in the book. We are talking joint limits,
% orientation locking, and a virtual wall...
n = [2 -1 2]';
[q_des_cB,~, le_cB,cond_ang_Cb,cond_lin_Cb,iso_ang_Cb,iso_lin_Cb] = IK_part_cB(M,S,q0,J_limits,d_tool,p_goal,n);
final_q = rad2deg(q_des_cB)

figure(14)
plot(le_cB)
    ylim([0 1.2*max(le_cB)])
     title('Linear Error')
    ylabel('error [m]')
    xlabel('iteration')

figure(15)
lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'column';
show(lbr,q_des_cB, 'Frames', 'off');
hold on;
T1 = FK_space(M,S,q_des_cB);
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

figure(16)
plot(cond_ang_Cb)
hold on 
plot(cond_lin_Cb)
title('Condition number Cb')
hold off
legend('Cond angular Cb','Cond linear Cb')

figure(17)
plot(iso_ang_Cb)
hold on 
plot(iso_lin_Cb)
title('Isotropy number Cb')
hold off 
legend('Iso angular Cb','Iso linear Cb')


%%
% Discusion - I'll be honest I was surprised with this one. Despite j6 hitting its
% joint limit, the more constrained system was able to perform better.
% Perhaps the combination of the constraints increased the robusteness of
% the system. Regardless, converging in 49 iterations is pretty efficient.

%% Part d - Configs
% We tested 3 separate configs, in a row. Sounds like fun.

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

figure(18)
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

figure(19)
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

figure(20)
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

figure(21)
le_cB = [le1 le2 le3];
plot(le_cB), hold on;
xline([length(le1) length(le1)+length(le2)], ':');
    hold off;
    title('Linear Error: All Restraints')
    ylim([0 1.2*max(le_cB)])

%% final plot
figure(22)
plot(le_a); hold on
plot(le_b)
plot(le_cA)
plot(le_cB)
    title('Linear Error Comparison')
    ylabel('error [m]')
    xlabel('iteration')
    legend('Part a','Part b','Part c-a','Part c-b')
