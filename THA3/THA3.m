%% THA3 - PA1 & PA2
%   Programming Assignment 3, the Emperor Strikes back
% 
% Authors: Pranav Kalyani, pk7683; Michael Normand, man2837 
%
%
% Date: May 01 2022

clear; clc; close all
addpath('src')

%%
%   Data is contained as a struct
%   *getDataset* is in Appendix
df = getDataset('a');

%% Problem 1 - Point Set Registration
%   We created a _registerPoints()_ function for general point calibration.
%   This function
%
% <include>src/registerPoints.m</include>
%

%% Problem 2 - Pivot Calibration
%   The following function performs Pivot Calibration for both EM sensor and
%   Optical Sensor
%
% <include>src/pivotCalibration.m</include>
%

%% Problem 3 - C_{i, expected}
%   We used the following script to calculate the frame transformations
%   from a given dataset.
% 
% <include>src/frameTransforms.m</include>
%

%%
% *a.) Compute Fd* 
%   Using _frameTransforms()_ we can directly get the Fd or Fa transforms
%

Fd = frameTransforms(df);

% Check for errors
disp('Checking Fd Validity:')
disp('frame:  Error')

% setup loop vars
a = df.calbody.d;
A = df.calreadings.D;
Ta = eye(4);
TA = eye(4);

for k = 1:size(A,3)
    % error per frame
    frame_err = 0;

    for i = 1:height(a)
        % convert position to translation matrix
        Ta(1:3,4) = a(i,:);
        TA(1:3,4) = A(i,:,k);

        % calculate difference between expected an actual
        err = TA - Fd(:,:,k)*Ta;
        frame_err = frame_err + norm(err(1:3,4));
    end
    fprintf('\t%d: \t% 0.3f\n',k,frame_err)
end

%%
% *b.) Compute Fa* 
%   same as before
Fa = frameTransforms(df, 0);

% Check for errors
disp('Checking Fa Validity:')
disp('frame:  Error')

% setup loop vars
a = df.calbody.a;
A = df.calreadings.A;
Ta = eye(4);
TA = eye(4);

for k = 1:size(A,3)
    % error per frame
    frame_err = 0;

    for i = 1:height(a)
        % convert position to translation matrix
        Ta(1:3,4) = a(i,:);
        TA(1:3,4) = A(i,:,k);

        % calculate difference between expected an actual
        err = TA - Fa(:,:,k)*Ta;
        frame_err = frame_err + norm(err(1:3,4));
    end
    fprintf('\t%d: \t% 0.3f\n',k,frame_err)
end


%%
% *c.) Compute C_exp*
%   The following is included in the _logOutput()_ function
%
%   c = df.calbody.c;
%   n_c = height(c);
%   n_frames = size(Fd, 3);
%   
%   c_i = eye(4);
%   C_exp = zeros(n_c,3,n_frames);
%   for k = 1:n_frames
%       for i = 1:n_c
%           c_i(1:3, 4) = c(i,:);
%           
%           C_i = Fd(:,:,k)\Fa(:,:,k)*c_i;
%           
%           C_exp(i,:,k) = C_i(1:3,4);
%       end
%   end
%

%% 
% *d.) Output as file*
%   _logOutput()_ handles all of the output requirements, includeing pivot
%   calibration
%
% <include>src/logOutput.m</include>
%
% .........................................................................

logOutput(df);

%% Problem 4 - Pivot Calibration w/ EM
%  Pivot Calibration using _pivotCalibration()_
post_pos_EM = pivotCalibration(df)

%% Problem 5 - Pivot Calibration w/ Optics
%  Pivot Calibration using _pivotCalibration()_
post_pos_OPT = pivotCalibration(df, 0)

%% PA2.1 - hand-eye calibration algorithm 
%  solves for unknown transformation X from camera frame to robot's end
%  effector frame
%
% <include>src/ax_xb.m</include>
%
%%
% calibration without noise
 
 q_Robot_config=[
    -0.321, 0.087, 0.682, 0.651
    -0.747, 0.431, -0.061, 0.502
    -0.361, 0.310, 0.045, 0.878
    -0.676, 0.414, -0.038, 0.608
    -0.772, 0.256, 0.204, 0.545
    -0.070, 0.530, 0.113, 0.838
    -0.069, 0.582, 0.113, 0.803
    -0.100, 0.470, 0.167, 0.861
    -0.129, 0.463, 0.221, 0.849
    0.137, 0.433, 0.165, 0.876];


q_camera_config=[
      0.608, -0.319, 0.723, -0.077
      0.205, 0.383, -0.730, 0.527
      -0.064, -0.309, 0.948, -0.034
      -0.178, -0.379, 0.803, -0.424
      -0.104, 0.313, -0.778, 0.534
      -0.078, -0.536, 0.809, 0.226
      -0.096, -0.587, 0.775, 0.213
      -0.007, -0.502, 0.840, 0.205
      0.046, -0.513, 0.839, 0.173
      0.003, -0.465, 0.773, 0.431];

t_Robot_config=[
      0.261, -0.369, 0.488
      0.100, 0.446, 0.506
      -0.099, -0.166, 0.677
      0.050, 0.292, 0.299
      -0.172, 0.233, 0.287
      0.379, -0.057, 0.557
      0.374, -0.058, 0.347
      0.263, -0.040, 0.398
      0.304, 0.008, 0.380
      0.254, -0.132, 0.391];
  
  t_camera_config=[
      0.252, 0.112, 0.577
      -0.282, 0.037, 0.723
      -0.312, 0.191, 0.836
      -0.076, -0.074, 0.595
      0.188, 0.023, 0.727
      0.048, 0.104, 0.549
      -0.039, -0.007, 0.363
      0.033, -0.019, 0.452
      0.103, -0.016, 0.401
      0.069, -0.013, 0.463];
  
       q_Robot_config2=[
    -0.321, 0.087, 0.682, 0.651
    -0.747, 0.431, -0.061, 0.502
    -0.361, 0.310, 0.045, 0.878
    -0.676, 0.414, -0.038, 0.608
    -0.772, 0.256, 0.204, 0.545];


q_camera_config2=[
      0.608, -0.319, 0.723, -0.077
      0.205, 0.383, -0.730, 0.527
      -0.064, -0.309, 0.948, -0.034
      -0.178, -0.379, 0.803, -0.424
      -0.104, 0.313, -0.778, 0.534];

t_Robot_config2=[
      0.261, -0.369, 0.488
      0.100, 0.446, 0.506
      -0.099, -0.166, 0.677
      0.050, 0.292, 0.299
      -0.172, 0.233, 0.287];
  
  t_camera_config2=[
      0.252, 0.112, 0.577
      -0.282, 0.037, 0.723
      -0.312, 0.191, 0.836
      -0.076, -0.074, 0.595
      0.188, 0.023, 0.727];
      
      

  ansPa2_1 = ax_xb(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config )
  
%% PA2.2 - hand-eye calibration algorithm 
%  solves for unknown transformation X from camera frame to robot's end
%  effector frame with random noise 
%
% <include>src/ax_xb_noisy.m</include>
%
%%
% part one calibration with noise
  
 ansPa2_2prt1 = ax_xb_noisy(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config )
%
%% 
% part two compare noise data vs noise free data

   error = abs(ansPa2_1-ansPa2_2prt1).^2;
   MSE = ((error(:))/numel(ansPa2_1))'
   MSE_sum = sum((error(:))/numel(ansPa2_1))

% as you can see from the results there is alot of error created by the 
%random noise introduced in the sensor and robot configuration values

   
%% 
% part three compare no noise data vs noise data for half sets of the
% configuration 
   ansPa2_2prt3_free = ax_xb(q_Robot_config2, q_camera_config2,t_Robot_config2,t_camera_config2 );
   ansPa2_2prt3_noise = ax_xb_noisy(q_Robot_config2, q_camera_config2,t_Robot_config2,t_camera_config2 );
   
   error2 = abs(ansPa2_2prt3_free-ansPa2_2prt3_noise).^2;
   MSE2 = ((error2(:))/numel(ansPa2_2prt3_free))'
   MSE_sum2 = sum((error2(:))/numel(ansPa2_2prt3_free))
   
   % The results show a significant increase in error per element and also as a
   % sum. The data is significantly affected by the noise because 
   % the smaller data set is more prone to be bothered 
   % by the noise since the system is fairly sensitive 
   % if the data set is small 
   %
   

%% Appendix
%   1. getDataset()
%
% <include>src/getDataset.m</include>
%
%   2. quaternion2rotm()
%
% <include>src/quaternion2rotm.m</include>
%
%   3. skewify
%
% <include>src/skewify.m</include>
%
%   4. Rotation
%
% <include>src/Rotation.m</include>