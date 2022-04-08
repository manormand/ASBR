%% HA2.m
% Handwritten Assignment 2 - Supplemental code
%
%%
clear; clc; close all;
addpath("src\") % adds src directory containing functions
% urdf_file = 'urdf/kuka_iiwa7_URDF.xacro';
% robot = getRobotFromURDF(urdf_file);

%% UnitTests
% All funcitons in the src folder have at least one Unit Test in order to
% validate their functionality. Most examples are from Lecture or Modern
% Robotics with a few exceptions. The UnitTest class is included at the
% bottom of this report detailing each test

results = runtests('PA2UnitTests.m');
disp(results)

%% robot params
% These were defined from the paper TODO: Insert paper
% d* is the z-length between joint segments, where joints are grouped into
% couples of z-axis and y-axis revolute joints
%
% * dbs is between base and j2/j3 (shoulder)
% * dse is between j2/j3 and j4/j5 (elbow)
% * dew is between j4/j5 and j6/j7 (wrist)
% * dwf is between j6/j7 and end effector (finger)

% link lengths [m]
dbs = 0.340;
dse = 0.400;
dew = 0.400;
dwf = 0.126;

% home position
M = eye(4); M(3,4) = dbs+dse+dew+dwf;

% arbitrary joint positions
q = deg2rad([20 -10 30 0 -30 45 0]');

%% PA.b - Space Form Forward Kinematics
% The function FK_space accepts any serial chain specified in space-form
% screw axes
%
% <include>src/FK_space.m</include>
%
% Kuka LBR example:

% Space form Screw Axes
S = [   0    0 0       0 0            0 0;
        0    1 0      -1 0            1 0;
        1    0 1       0 1            0 1;
        0 -dbs 0 dbs+dse 0 -dbs-dse-dew 0;
        0    0 0       0 0            0 0;
        0    0 0       0 0            0 0];

T_space = FK_space(M,S,q, true);

fprintf('\tT_space:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_space.')

%% PA.c - Body Form Forward Kinematics
% The function FK_body accepts any serial chain specified in body-form
% screw axes
%
% <include>src/FK_body.m</include>
%
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
% *Space Form* Jacobian is calculcated directly from the Space-Form screw
% axes, _S_, and the joint positions, _q_
%
% <include> src/J_space.m </include>
%
% Kuka LBR example:
Jacob_space = J_space(S,q);

fprintf('\tJ_space:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_space.')

%%
% *Body Form* Jacobian is similarly calculated from the Body-From screw
% axes, _B_, and the joint positions, _q_
%
% <include> src/J_space.m </include>
%
% Kuka LBR example:
Jacob_body = J_body(B,q);

fprintf('\tJ_body:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_body.')

%% PA.h - Inverse Kinematics
% Inverse kinematics uses the Newton Raphson Method to calculate the
% joint positions required to achieve an end effector pose, specified by
% Tsd
%
% <include>src/J_inverse_kinematics.m</include>
%
% Kuka LBR example:

% TODO
%% PA.i - Transpose Kinematics
% Transpose kinematics uses closed loop control with a weighted matrix
% controller in order to find the joint positions required to achieve an
% end effector pose, specified by Tsd
%
% <include>src/J_transpose_kinematics.m</include>
%
% Kuka LBR example:

T_sd = T_space; % target pose from Forward-Kinematics example

q_guess = q / norm(q) + rand(size(q)); % randomized initial guess
q_ik = J_inverse_kinematics(M,B,q_guess,T_sd);

T_J_inv_kin = FK_body(M,B,q_ik);

disp('=============================================')
fprintf('\tTarget Pose:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_sd.')

disp('=============================================')
fprintf('\tDerived Pose:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T_J_inv_kin.')
%% UNIT TEST Documentation
% Unit Test class inherits from the matlab UnitTests class.
%
% <include>PA2UnitTests.m</include>