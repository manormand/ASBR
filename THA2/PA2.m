%% PA2.m
% Programming Assignment 2 - Electric Boogaloo
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

%% Robot Karams
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
% screw axes. FK_space returns the end effector pose and it also plots the 
% serial chain in 3d space if specified. 
%
% <include>src/FK_space.m</include>
%
% ------------------------------------------------------------------------
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

disp('=============================================')
fprintf('T_space:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', T_space.')
disp('=============================================')
%% PA.c - Body Form Forward Kinematics
% The function FK_body accepts any serial chain specified in body-form
% screw axes. This function returns the end effector pose only.
%
% <include>src/FK_body.m</include>
%
% ------------------------------------------------------------------------
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

disp('=============================================')
fprintf('T_body:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', T_body.')
disp('=============================================')

%% PA.e - Jacobians
% *Space Form* Jacobian is calculcated directly from the Space-Form screw
% axes, _S_, and the joint positions, _q_.
%
% <include> src/J_space.m </include>
%
% ------------------------------------------------------------------------
%
% Kuka LBR example:
Jacob_space = J_space(S,q);

disp('=============================================')
fprintf('J_space:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_space.')
disp('=============================================')%%
% *Body Form* Jacobian is similarly calculated from the Body-From screw
% axes, _B_, and the joint positions, _q_.
%
% <include> src/J_space.m </include>
%
% ------------------------------------------------------------------------
%
% Kuka LBR example:
Jacob_body = J_body(B,q);

disp('=============================================')
fprintf('J_body:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f % .3f % .3f % .3f ]\n', ...
        Jacob_body.')
disp('=============================================')
%% PA.f - singularity.m
% The function singularity.m accepts any serial chain specified in space-form
% screw axes and a set of angles q and return whether the configuration is
% in a singularity 
%
% <include> src/singularity.m </include>
%
% Kuka LBR example:
sing = singularity(S, q); 
if sing 
   disp('We are in a singularity')
else
    disp('No singularity')
end



%% PA.g - Manipulabilty 
% *ellipsoid_plot_angular* The function ellipsoid_plot_angular.m accepts any serial chain specified in space-form
% screw axes and a set of angles q and returns a angular manipulabilty
% elipse plot
% 
% <include> src/ellipsoid_plot_angular.m </include>
%
% Kuka LBR example:
ellipang = ellipsoid_plot_angular(S, q);

%%
% *ellipsoid_plot_linear* The function ellipsoid_plot_linear.m accepts any serial chain specified in body-form
% screw axes and a set of angles q and returns a linear manipulabilty
% elipse plot
%
% <include> src/ellipsoid_plot_linear.m </include>
%
% Kuka LBR example:
ellip_lin = ellipsoid_plot_linear(S, q);

%%
% *J_isotropy* The function J_isotropy.m accepts any serial chain specified
% in space-form and body-form screw axes and a set of angles q and returns isotropy
%
% <include> src/J_isotropy.m </include>
%
% Kuka LBR example:
J_iso = J_isotropy(S,B,q);

disp('=============================================')
fprintf('\tJ_isotropy:\n')
disp(J_iso)
disp('=============================================')

%%
% *J_condition* The function J_condition.m accepts any serial chain specified
% in space-form and body-form screw axes and a set of angles q and returns
% condition number
%
% <include> src/J_condition.m </include>
%
% Kuka LBR example:
J_con = J_condition(S,B,q);

disp('=============================================')
fprintf('\tJ_condition:\n')
disp(J_con)
disp('=============================================')

%%
% *J_ellipsoid_volume* The function J_ellipsoid_volume.m accepts any serial chain specified
% in space-form and body-form screw axes and a set of angles q and returns
% volume of the two ellipsoids 
%
% <include> src/J_ellipsoid_volume.m </include>
%
% Kuka LBR example:
J_vol = J_ellipsoid_volume(S,B,q);

disp('=============================================')
fprintf('\tJ_ellipsoid_volume:\n')
disp(J_vol)
disp('=============================================')








%% PA.h - Inverse Kinematics
% Inverse kinematics uses the Newton Raphson Method to calculate the
% joint positions required to achieve an end effector pose, specified by
% Tsd.
%
% <include>src/J_inverse_kinematics.m</include>
%
% ------------------------------------------------------------------------
%
% Kuka LBR example:

Tsd = T_space; % target pose from Forward-Kinematics example

q_guess = rand(size(q)); % randomized initial guess
q_ik = J_inverse_kinematics(M,B,q_guess,Tsd);

T_J_inv_kin = FK_body(M,B,q_ik);

disp('=============================================')
fprintf('Target Pose:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', Tsd.')

fprintf('J_inverse_kinematitcs:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', T_J_inv_kin.')
disp('=============================================')

%% PA.i - Transpose Kinematics
% Transpose kinematics uses closed loop control with a weighted matrix
% controller in order to find the joint positions required to achieve an
% end effector pose, specified by Tsd. Because this method takes longer to
% become accurate, we decreased the random number by a factor of 10.
%
% <include>src/J_transpose_kinematics.m</include>
%
% ------------------------------------------------------------------------
%
% Kuka LBR example:

q_guess = q + 0.1*rand(size(q)); % randomized initial guess
q_ik = J_transpose_kinematics(M,B,q_guess,Tsd);

T_J_tran_kin = FK_body(M,B,q_ik);

disp('=============================================')
fprintf('Target Pose:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', Tsd.')

fprintf('J_transpose_kinematics:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', T_J_tran_kin.')
disp('=============================================')


%% PA.j - Redundancy Resolution
% Adding the manipulability as a second objective to the inverse kinematics
% slows down the algorithm, but it is still fairly consistent in results.
%
% <include>src/redundancy_resolution</include>
%
% ------------------------------------------------------------------------
%
% Kuka LBR example:


q_guess = rand(size(q)); % randomized initial guess
q_ik = redundancy_resolution(M,B,q_guess,Tsd);

T_red_res = FK_body(M,B,q_ik);

disp('=============================================')
fprintf('Target Pose:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', Tsd.')

fprintf('redundancy_resolution:\n')
fprintf('\t[ % .3f % .3f % .3f % .3f ]\n', T_red_res.')
disp('=============================================')

%% PA.m - Kuka Graphical Sim
% Using the MATLAB Robotics toolbox we can also directly import the KUKA
% arm

lbr = importrobot('iiwa14.urdf'); % 14 kg payload version
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';
config = randomConfiguration(lbr);
show(lbr,config);


%% UNIT TEST Documentation
% Unit Test class inherits from the matlab UnitTests class.
%
% <include>PA2UnitTests.m</include>