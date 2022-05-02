%% THA3 - PA1
%   Programming Assignment 3.1, the Emperor Strikes back
% 
% Authors: Pranav Kalyani, pk7683
%
%          Michael Normand, man2837 
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