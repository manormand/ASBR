%% PA2 - Animation
% Animates the KUKA LBR IIWA
clear;clc;close all

% load in robot config
figure()
lbr = importrobot('iiwa7.urdf'); % 7 kg payload version
lbr.DataFormat = 'column';

q = deg2rad([20 50 30 50 -30 60 0]');

show(lbr,q);
    xlim([-1 1]), ylim([-1 1]), zlim([-0.25 1.75])
    title('KUKA LBR iiwa 7')

%% Read Robot Tree for M, B
n = 7;

% Init containers
M = lbr.Bodies{1,1}.Joint.JointToParentTransform; % Base frame
B = zeros(6,n);
links = zeros(3,n);

for i = 1:n
    joint_i = lbr.Bodies{1,i+1}.Joint;
    M_i = joint_i.JointToParentTransform;
    M = M*M_i;
    
    rotm_i = M(1:3,1:3);
    B(1:3,i) = rotm_i*joint_i.JointAxis';

    links(:,i) = M_i(1:3,4);
end

% Add the last transform
M_i = lbr.Bodies{1,n+2}.Joint.JointToParentTransform;
M = M_i;

B


%% actual
dbs = 0.340;
dse = 0.400;
dew = 0.400;
dwf = 0.126;

% home position
M = eye(4); M(3,4) = dbs+dse+dew+dwf;

B = [   0           0 0        0 0   0 0;
        0           1 0       -1 0   1 0;
        1           0 1        0 1   0 1;
        0 dse+dew+dwf 0 -dew-dwf 0 dwf 0;
        0           0 0        0 0   0 0;
        0           0 0        0 0   0 0];

%% Inverse Kinematics

%% Transpose Kinematics