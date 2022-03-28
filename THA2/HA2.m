%% HA2.m
% Handwritten Assignment 2 - Supplemental code
%
%%
clear; clc; close all;
addpath("src\") % adds src directory containing functions

%% Problem 4

% Set givens
syms L

th1 = 0;
th2 = 0;
th3 = pi/2;
th4 = L;

dth1 = 1;
dth2 = 1;
dth3 = 1;
dth4 = 1;

% set Screw Axes
w1 = [sin(th2)*sin(th3); -sin(th2)*cos(th3); cos(th3)];
w2 = [cos(th3); sin(th3); 0];
w3 = [0 0 1]';
w4 = [0 0 0]';

v1 = [ cos(th2)*(L*cos(th3) + th4);
        -L*cos(th2)*sin(th3);
       sin(th2)*sin(th3)*(2*L*cos(th3) + th4)];
v2 = [0; 0; L*(cos(th3)^2 - sin(th3)^2) + th4*cos(th3)];
v3 = [0; 0; 0];
v4 = [0; 1; 0];

% Calc T_matrices
M = sym(eye(4));
M(1:3,4) = [0; L; L];
T1 = screwAxis2TMat(w1,v1,th1);
T2 = screwAxis2TMat(w2,v2,th2);
T3 = screwAxis2TMat(w3,v3,th3);
T4 = screwAxis2TMat(w4,v4,th4);

T = M*T1*T2*T3*T4

%% what