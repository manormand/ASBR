function logOutput(df)
% logOuptput log the calculated outputs to an output file
%   Use the pivot calibration to find the post location and then use least
%   squares to find the calibration object position. Finally output into
%   the data_out folder in specified format

% get transforms
Fd = frameTransforms(df);
Fa = frameTransforms(df, 0);

% init c and loop vars
c = df.calbody.c;
n_c = height(c);
n_frames = size(Fd, 3);

c_i = eye(4);
C_exp = zeros(n_c,3,n_frames);

% Calc C_exp at every point and frame
for k = 1:n_frames
    for i = 1:n_c
        c_i(1:3, 4) = c(i,:);    
        C_i = Fd(:,:,k)\Fa(:,:,k)*c_i;
        
        C_exp(i,:,k) = C_i(1:3,4);
    end
end

% calc post positions using pivot calibration
post_pos_EM = pivotCalibration(df);
post_pos_OPT = pivotCalibration(df, 0);

% open file + add headers
filename = ['pa1-' df.id '-output-1.txt'];
filepath = fullfile('data_out', filename);

file_id = fopen(filepath,'w');

fprintf(file_id, '%d, %d, %s\n', n_c, n_frames, filename);
fprintf(file_id, '% .2f, % .2f, % .2f\n', post_pos_EM');
fprintf(file_id, '% .2f, % .2f, % .2f\n', post_pos_OPT');

% add data to file
for k = 1:n_frames
    fprintf(file_id, '% .2f, % .2f, % .2f\n', C_exp(:,:,k)');
end

fclose(file_id);