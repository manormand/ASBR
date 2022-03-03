%% ASBR THA1 Programming Assignemnt
%
% Authors: Pranav Kalyani and Michael Normand
%
% Date: 2022-MAR-03
%
%%
% This script relies on the class *Rotation*, defined in _Rotation.m_ 
% (Included at the end).
% A UnitTests class is also included, *RotationTests*, which packages the 
% tests from Problem 1 and 2 into a single class.
%
% To run RotationTests: while in the same folder as _Rotation.m_ and 
% _RotationTests.m_, call:
%%
%   runtests()
%
%% Problems 1 + 2 Info
% This section verifies that the static *Rotation* method produces the
% correct output when compared to a predefined representation.
%
% The rotation was sourced from the _Modern Robotics_ Example
% 3.12. Defined below is the rotation matrix and its equivalent
% representations, derived by hand or given with the textbook. 
%
clear; clc; close all

% setup globals for Problems 1 and 2
tolerance = 0.001;
pass_or_fail = ["Fail", "Pass"];

% Init equivalent rotation representations
rotm = [  0.866 -0.250 0.433;
          0.250  0.967 0.058;
         -0.433  0.058 0.899 ];

axis_ = [0 0.866 0.5]';
angle = deg2rad(30);
axangle = [angle; axis_];

q = [0.9659 0 0.2242 0.1294]';

ZYZ = [-3.0084 -0.4523 -3.0084];
RPY = [ 0.2810  0.4478  0.0644];

%% Problem 1 - rotm2*
%%
% *a) Rotation Matrix to Axis Angle*

% Calculate axangle with Rotation class
[w_calc, th_calc] = Rotation.rotm2axangle(rotm);
axangle_calc = [th_calc; w_calc];

% check if calculated axis and angle is within tolerance of actual
check = abs(axangle - axangle_calc) < tolerance;
check_1a = all(check) + 1; % results in 1 for fail, 2 for pass

%%
% *b) Rotation Matrix to Quaternion*

% Calculate quaternion
q_calc = Rotation.rotm2quaternion(rotm);

% check if calculated quaternion is within tolerance of actual
check = abs(q - q_calc) < tolerance;
check_1b = all(check) + 1;

%%
% *c) Rotation Matrix to Euler Angles*

% Calculate Euler angles with Rotation class
ZYZ_calc = Rotation.rotm2euler(rotm,"ZYZ");
RPY_calc = Rotation.rotm2euler(rotm,"RPY");

% Check if calculated angles are within tolerance of actual
check_zyz = abs(ZYZ - ZYZ_calc) < tolerance;
check_rpy = abs(RPY - RPY_calc) < tolerance;

check_1c_zyz = all(check_zyz) + 1;
check_1c_rpy = all(check_rpy) + 1;

%%
% *Printout results*
disp('----- THA1 Programming assignment -----')
disp('Problem 1 - rotm2*')
fprintf('\tTest: rotm2axangle...%s\n', pass_or_fail(check_1a))
fprintf('\tTest: rotm2quaternion...%s\n', pass_or_fail(check_1b))
fprintf('\tTest: rotm2euler(ZYZ)...%s\n', pass_or_fail(check_1c_zyz))
fprintf('\tTest: rotm2euler(RPY)...%s\n', pass_or_fail(check_1c_rpy))

%% Problem 2 - *2rotm
%%
% *a) Axis Angle to Rotation Matrix*

% Calculate rotm with Rotation class
rotm_calc = Rotation.axangle2rotm(axis_,angle);

% check if calculated rotation matrix is within tolerance of given
check = abs(rotm - rotm_calc) < tolerance;
check_2a = all(reshape(check,1,9)) + 1;

%%
% *b) Quaternion to Rotation Matrix*

% Calculate quaternion with Rotation class
rotm_calc = Rotation.quaternion2rotm(q);

% check if calculated rotation matrix is within tolerance of given
check = abs(rotm - rotm_calc) < tolerance;
check_2b = all(reshape(check,1,9)) + 1;

%%
% *Printout results*
disp('Problem 2 - *2rotm')
fprintf('\tTest: axangle2rotm...%s\n', pass_or_fail(check_2a))
fprintf('\tTest: quaternion2rotm...%s\n', pass_or_fail(check_2b))

%% Problem 3 - A Loose Screw
% For problem 3, the user defines the parameters in the indicated section.
% After which, this program plots the transformation and returns the final
% configuration _T1_ and its inverse expressed with the Screw Axis _S1_ and
% the distance $\theta_1$.
clear

%%
% *----------------- User Defined Parameters -----------------*
q = [0 2 0]';       % Position of Screw Axis
s = [0 0 1]';       % Screw Axis Direction
h = 2;              % Pitch
theta = pi;         % Total Rotation

T = eye(4); T(1,4) = 2; % initial config of Rigid Body

dth = 1; % rotation speed (maybe don't change this)

%%
% *----------------- Simulate Screw Motion -----------------*

% create screw axis
w = s*dth;
v = cross(-w,q) + h*w;

w_skew = Rotation.skewify(w);

% ~Semi~ Continuous Containers
dt = 0.05;
theta_vect = theta * (0:dt:1);
p = zeros(3,length(theta_vect));

% Intermediate Containers
dt = 0.25;
theta_int_vect = theta * (0:dt:1);
T_mats = zeros(4,4,length(theta_int_vect));
count = 1;

% Calc Screw axis through time intervals
for i = 1:length(theta_vect)
    th = theta_vect(i);

    % Create T_S + T_i
    pos    = ( eye(3)*th + (1-cos(th))*w_skew + (th - sin(th))*w_skew^2 )*v;
    rotm   = Rotation.axangle2rotm(w, th);

    T_S = [rotm pos; 0 0 0 1];
    T_i = T_S*T;

    p(:,i) = T_i(1:3,4);

    % During intermediate intervals record T_i
    if any(theta_int_vect==th)
        T_mats(:,:,count) = T_i;
        count = count+1;
    end
end

%%
% *----------------- T1 + S1 Calculation -----------------*
T1 = T_mats(:,:,end);

% Invert T1 (T_01) to get T10 (T_10)
T10 = inv(T1);
rotm_b = T10(1:3,1:3);
pos_b  = T10(1:3,4);

% check if rotm is identity
id_check = abs(rotm_b - eye(3)) > 0.001;
if all(id_check)
    w_b = [0 0 0]';
    v_b = pos_b/norm(pos_b);
    th_b = norm(pos_b);
    q_b = pos_b;
else
    % get theta and w from rotm
    [w_b, th_b] = Rotation.rotm2axangle(rotm_b);
    wb_skew = Rotation.skewify(w_b);
    
    % solve for v0
    G_inv = (1/th_b)*eye(3) - 0.5*wb_skew + ...
                ((1/th_b) - 0.5*cot(th_b/2))*wb_skew^2;
    v_b = G_inv*pos_b;

    % solve for q
    q_b = pos_b / (2*sin(th_b/2));
end

S1 = [w_b; v_b];

%%
% *----------------- Print _Results_ -----------------*
disp('Problem 3: Screwed Up') % I think I am nailing these puns
fprintf('\tT1:\n')
fprintf('\t\t[ % .3f % .3f % .3f % .3f ]\n', T1.')

fprintf('\tS1:\n')
fprintf('\t\t[ % .3f ]\n', S1)

fprintf('\ttheta1: \n\t\t%.3f\n', th_b)

%%
% *----------------- Plotting -----------------*

% Screw axis representation
screw_ax_length = 20;
screw_ax = [(q_b-w_b*screw_ax_length) q_b (q_b+w_b*screw_ax_length)];

% Init plot
figWidth = 960; % pixels
figHeight = 720;
rect = [0 50 figWidth figHeight];
fig = figure('OuterPosition',rect);

plot3(screw_ax(1,:), screw_ax(2,:), screw_ax(3,:), 'm:', LineWidth=3)
    title('Screw That!!!')
    xlabel('X'), ylabel('Y'), zlabel('Z')
    axis equal
    hold on
    grid on

% Plot Screw Path
X = p(1,:);
Y = p(2,:);
Z = p(3,:);

plot3(X,Y,Z,'k', LineWidth=1.5)

% Plot b_i Frames
% init axes
ax_len = 0.35;
x_ax = [ax_len 0 0 1]';
y_ax = [0 ax_len 0 1]';
z_ax = [0 0 ax_len 1]';
axes = [x_ax, y_ax, z_ax];

colors = ["b","r","g"];

for i = 1:5
    % get current Transform
    T_i = T_mats(:,:,i);
    p_i = T_i(1:3,4);
    
    % plot origin
    plot3(p_i(1), p_i(2), p_i(3), 'ro')

    for j = 1:3
        % transform ax
        ax = T_i*axes(:,j);

        % plot ax
        X = [p_i(1) ax(1)];
        Y = [p_i(2) ax(2)];
        Z = [p_i(3) ax(3)];
        plot3(X,Y,Z,colors(j), LineWidth=1)
    end
end

% Final plot edits
xlim([-3 3]), ylim([-1 5]), zlim([0 7])
legend(["$S_1$","Path","$\{b_i\} O$", ...
    "$\{b_i\} \hat{x}$", "$\{b_i\} \hat{y}$", "$\{b_i\} \hat{z}$"], ...
    'Interpreter','latex', 'FontSize',14)
hold off

%% Rotation Class
% The *Rotation* class includes the static methods used throughout this
% assignment. 
%%
% <include>Rotation.m</include>