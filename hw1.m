%% ASBR HW1 Programming Assignment
% In this assignment we extensively use the class Rotation, defined in a
% separate file.

%%
clear; clc

%% Problem 1
% define givens
theta = deg2rad(30);
w = [0; 0.866; 0.5];

% Get Euler Angles
rotm = Rotation.axangle2rotm(w,theta);
Phi = Rotation.rotm2euler(rotm, 'RPY');

% check validity
Phi_act = [0.0644047, 0.4478289, 0.2810408]';
tolerance = 0.001;
rotm_check = abs(Phi-Phi_act) <= tolerance