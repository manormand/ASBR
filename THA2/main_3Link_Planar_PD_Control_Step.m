% main_3Link_Planar_PD_Control_Step.m
% Author: Michael Normand
% Date: 2021 April 23
% Description: Run a simulation of a 3dof planar arm with PD control and
% step inputs
clear; clc; close all;

%% Define System Parameters
% Init System Params
L1 = 0.5;       % Link_i length
L2 = 0.5;
L3 = 0.5;
L1c = 0.25;     % Link_i CoM displacement
L2c = 0.25;
L3c = 0.25;
m1 = 1;         % Link_i mass
m2 = 1;
m3 = 1;
I1 = 1;         % Link_i Moment of Inertia
I2 = 1;
I3 = 1;
g = 9.81;       % Gravity Acceleration
B = 0.0;        % Damping Coefficient

% Init Estimated System Params
L1e = L1*1.0;
L2e = L2*1.0;
L3e = L3*1.0;
L1ce = L1c*1.0;
L2ce = L2c*1.0;
L3ce = L3c*1.0;
m1e = m1*1.0;
m2e = m2*1.0;
m3e = m3*1.0;
I1e = I1*1.0;
I2e = I2*1.0;
I3e = I3*1.0;

%% Define Control/Sim Parameters
% Init Timing Params
Hz = 1000;
dt = 1/Hz;

t0 = 0;
tf = 4;
time = t0:dt:tf;

% Controller Params
w = [25; 15; 10];
zeta = 0.7;
Kp = w.^2;
Kd = 2*zeta*w;

% Desired input
th_d = deg2rad([20; 50; 20]);
ddth_d = [0; 0];

% Disturbance Force
F = zeros(2, 1);
% F = [100; 150];

%% Simulation
% Initial States
th   =  [0; 0; 0];
dth  =  [0; 0; 0];
ddth =  [0; 0; 0];

% Init data matrices
ddth_out = zeros(length(time), 3);
dth_out  = zeros(length(time), 3);
th_out   = zeros(length(time), 3);
u_out    = zeros(length(time), 3);

for ii = 1:length(time)
    % current time
    t = time(ii);
    
    % joint states
    q1 = th(1);
    q2 = th(2);
    q3 = th(3);
    dq1 = dth(1);
    dq2 = dth(2);
    dq3 = dth(3);
    
    % Controller Input
    u = -Kd.*dth + Kp.*(th_d - th);
    
    % Disturbance
    J = zeros(2, 3); % init jacobian
    J(1, 1) = - L2*sin(q1 + q2) - L1*sin(q1) - L3*sin(q1 + q2 + q3);
    J(1, 2) = - L2*sin(q1 + q2) - L3*sin(q1 + q2 + q3);
    J(1, 3) = -L3*sin(q1 + q2 + q3);
    J(2, 1) = L2*cos(q1 + q2) + L1*cos(q1) + L3*cos(q1 + q2 + q3);
    J(2, 2) = L2*cos(q1 + q2) + L3*cos(q1 + q2 + q3);
    J(2, 3) = L3*cos(q1 + q2 + q3);
    
    dist = J'*F;
    u = u + dist;
    tau1 = u(1); tau2 = u(2); tau3 = u(3);
    
    % Lagrange Dynamics
    ddq1 = (tau1 - tau2 + L1*L3c*m3*sin(q2 + q3) + L1*L2*m3*sin(q2) + L1*L2c*m2*sin(q2) - L1*g*m2*cos(q1) - L1*g*m3*cos(q1) - L1c*g*m1*cos(q1))/I1;
    ddq2 = -(I2*tau1 - I1*tau2 + I1*tau3 - I2*tau2 + I2*L1*L3c*m3*sin(q2 + q3) + I1*L2*g*m3*cos(q1 + q2) + I1*L2c*g*m2*cos(q1 + q2) + I1*L1*L2*m3*sin(q2) + I2*L1*L2*m3*sin(q2) + I1*L1*L2c*m2*sin(q2) + I2*L1*L2c*m2*sin(q2) - I1*L2*L3c*m3*sin(q3) - I2*L1*g*m2*cos(q1) - I2*L1*g*m3*cos(q1) - I2*L1c*g*m1*cos(q1))/(I1*I2);
    ddq3 = (I2*tau3 - I3*tau2 + I3*tau3 - I2*L1*L3c*m3*sin(q2 + q3) + I3*L2*g*m3*cos(q1 + q2) + I3*L2c*g*m2*cos(q1 + q2) + I3*L1*L2*m3*sin(q2) + I3*L1*L2c*m2*sin(q2) - I2*L2*L3c*m3*sin(q3) - I3*L2*L3c*m3*sin(q3) - I2*L3c*g*m3*cos(q1 + q2 + q3))/(I2*I3);
    
    ddth = [ddq1; ddq2; ddq3];
    
    % Numerical Integration
    dth = dth + ddth*dt;
    th  =  th +  dth*dt;
    
    % Record Data
    ddth_out(ii, :) = rad2deg(ddth);
     dth_out(ii, :) = rad2deg( dth);
      th_out(ii, :) = rad2deg(  th);
       u_out(ii, :) = u;
end

%% Plots
figure(1)
set(gcf,'position',[0 30 600 800])
subplot(3, 1, 1)
plot(time, th_out, 'Linewidth', 2)
hold on
yline(rad2deg(th_d(1)), ':', 'Linewidth', 1, 'color', [0 0.4450 0.7410])
yline(rad2deg(th_d(2)), ':', 'Linewidth', 1, 'color', [0.8500, 0.3250, 0.0980])
yline(rad2deg(th_d(3)), ':', 'Linewidth', 1, 'color', [0.9290, 0.6940, 0.1250])
hold off
title('Joint States')
ylabel('Angle [deg]')
legend('J1', 'J2', 'J3', 'J1^d', 'J2^d', 'J3^d')
set(gca, 'fontsize', 13)

subplot(3, 1, 2)
plot(time, dth_out, 'Linewidth', 2)
ylabel('Anglular Velocity [deg/s]')
set(gca, 'fontsize', 13)

subplot(3, 1, 3)
plot(time, ddth_out, 'Linewidth', 2)
ylabel('Acceleration [deg/s^2]')
xlabel('Time [sec]')
set(gca, 'fontsize', 13)

% Torque Plot
figure(2)
set(gcf, 'position', [600 630 600 200])
plot(time, u_out, 'Linewidth', 2)
legend('J1', 'J2', 'J3')
ylabel('Input Torque [Nm]')
xlabel('Time [sec]')
set(gca, 'fontsize', 13)

%% Dynamic Plot
figure(3)
set(gcf, 'position', [600 30 600 515])
% set up static plot elements
plot(0, 0, 'ro', 'LineWidth', 5); hold on;
plot([0 1.5], [0 0], 'r', 'LineWidth', 2); hold on;   % X Axis
plot([0 0], [0 1.5], 'g', 'LineWidth', 2); hold on;   % Y Axis
title('PD Control')
xlabel('x-axis'); ylabel('y-axis');
axis equal; axis([-1.5 1.5 -1.5 1.5]);
set(gca, 'fontsize', 13)

% set up dynamic plot elements
dummy_line = zeros(1, 4);   % 3 joints + EE
link_lines = plot(dummy_line, dummy_line, 'k', 'LineWidth', 3); hold on;
joint_dots = plot(dummy_line, dummy_line, 'ro', 'LineWidth', 3); hold off;

% run animation
step = 50;
PD_ani = VideoWriter('PD_control_ani.avi');
PD_ani.FrameRate = 1/50/dt;
open(PD_ani)
for ii = 1:step:length(time)
    % Get state vars
    q1 = deg2rad(th_out(ii, 1));
    q2 = deg2rad(th_out(ii, 2));
    q3 = deg2rad(th_out(ii, 3));
    
    % get positions
    pos = zeros(2, 4);
    pos(:, 2) = [L1*cos(q1); L1*sin(q1)];
    pos(:, 3) = pos(:, 2) + L2*[cos(q1+q2); sin(q1+q2)];
    pos(:, 4) = pos(:, 3) + L3*[cos(q1+q2+q3); sin(q1+q2+q3)];
    
    % plot data
    set(link_lines, 'XData', pos(1, :), 'YData', pos(2, :))
    set(joint_dots, 'XData', pos(1, :), 'YData', pos(2, :))
    
    pause(0.01)
    drawnow
    
    % save frames
    frame = getframe(gcf);
    writeVideo(PD_ani, frame);
end

close(PD_ani)