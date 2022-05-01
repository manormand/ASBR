%% PA1
clear; clc; close all

addpath('src')

%% Problem 1
%
% <include>src/registerPoints.m<\include>
%

%% Problem 2
%
% <include>src/pivotCalibration.m<\include>
%

%% Problem 3
% Calculate C_i
df = getDataset('b');

%%
% Script for calculating transforms for every frame
% 
% <include>src/frameTransforms.m<\include>
%

%%
% a.) `Compute Fd` with _frameTransforms()_
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
% b.) `Compute Fa` in similar manner
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
% c.) Compute C_exp
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
% d.) Output as file
logOutput(df);

%% Problem 4
%  Pivot Calibration using `pivotCalibration()`
post_pos_EM = pivotCalibration(df)

%% Problem 5
%  Pivot Calibration using `pivotCalibration()`
post_pos_OPT = pivotCalibration(df, 0)