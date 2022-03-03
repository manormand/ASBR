%% ASBR THA1_programming.m
% Script that checks the validity of Rotation class functions. For more
% information on the Rotation class and its functions, see Rotation.m
% documentation or call:
%%
%   help Rotation
%%
% Examples used are from either the book _Modern Robotics_ or using the
% Robotics Toolbox
%

clear;clc
disp('----- THA1 Programming assignment -----')

% setup globals for Problems 1 and 2
tolerance = 0.001;
pass_or_fail = ["Fail", "Pass"];

%% Problem 1
rotm_given = [  0.866 -0.250 0.433;
                0.250  0.967 0.058;
               -0.433  0.058 0.899 ];

%%
% *a) Rotation Matrix to Axis Angle*
%
% Using Example 3.12 from _Modern Robotics_

% define givens
w_act = [0 0.866 0.5]';
th_act = deg2rad(30);
axangle_act = [th_act; w_act];

% Calculate axangle with Rotation class
[w_calc, th_calc] = Rotation.rotm2axangle(rotm_given);
axangle_calc = [th_calc; w_calc];

% check if calculated axis and angle is within tolerance of actual
check = abs(axangle_act - axangle_calc) < tolerance;
check_1a = all(check) + 1; % results in 1 for fail, 2 for pass

%%
% *b) Rotation Matrix to Quaternion*
%
% Using rotm2quat from Robotics Toolbox

% Calculate quaternions
q_act = rotm2quat(rotm_given)';
q_calc = Rotation.rotm2quaternion(rotm_given);

% check if calculated quaternion is within tolerance of actual
check = abs(q_act - q_calc) < tolerance;
check_1b = all(check) + 1;

%%
% *c) Rotation Matrix to Euler Angles*
%
% Using rotm2eul from Robotics Toolbox

% Calculate angels using Matlab functions
ZYZ_act = rotm2eul(rotm_given, 'ZYZ');
RPY_act = rotm2eul(rotm_given, 'ZYX');

% Calculate Euler angles with Rotation class
ZYZ_calc = Rotation.rotm2euler(rotm_given,"ZYZ");
RPY_calc = Rotation.rotm2euler(rotm_given,"RPY");

% Check if calculated angles are within tolerance of actual
check_zyz = abs(ZYZ_act - ZYZ_calc) < tolerance;
check_rpy = abs(RPY_act - RPY_calc) < tolerance;

check_1c_zyz = all(check_zyz) + 1;
check_1c_rpy = all(check_rpy) + 1;

%%
% *Printout results*
disp('Problem 1 - rotm2*')
fprintf('\tTest: rotm2axangle...%s\n', pass_or_fail(check_1a))
fprintf('\tTest: rotm2quaternion...%s\n', pass_or_fail(check_1b))
fprintf('\tTest: rotm2euler(ZYZ)...%s\n', pass_or_fail(check_1c_zyz))
fprintf('\tTest: rotm2euler(RPY)...%s\n', pass_or_fail(check_1c_rpy))

%% Problem 2
%%
% *a) Axis Angle to Rotation Matrix*
%
% Using Example 3.12 from _Modern Robotics_

% define givens
w = [0 0.866 0.5]';
th = deg2rad(30);

% Calculate rotm with Rotation class
rotm_calc = Rotation.axangle2rotm(w,th);

% check if calculated rotation matrix is within tolerance of given
check = abs(rotm_given - rotm_calc) < tolerance;
check_2a = all(reshape(check,1,9)) + 1;

%%
% *b) Quaternion to Rotation Matrix*
%
% Using quat2rotm from Robotics Toolbox
q = rotm2quat(rotm_given);

% Calculate quaternion with Rotation class
rotm_calc = Rotation.quaternion2rotm(q);

% check if calculated rotation matrix is within tolerance of given
check = abs(rotm_given - rotm_calc) < tolerance;
check_2b = all(reshape(check,1,9)) + 1;

%%
% *Printout results*
disp('Problem 2 - *2rotm')
fprintf('\tTest: axangle2rotm...%s\n', pass_or_fail(check_2a))
fprintf('\tTest: quaternion2rotm...%s\n', pass_or_fail(check_2b))

%% Problem 3
% Transformation Matrix
