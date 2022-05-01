function logOutput(df)

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

% open file + add headers
filename = ['pa1-' df.id '-output-1.txt'];
filepath = fullfile('data_out', filename);

file_id = fopen(filepath,'w');

fprintf(file_id, '%d, %d, %s\n', n_c, n_frames, filename);


% add data to file
for k = 1:n_frames
    fprintf(file_id, '% .2f, % .2f, % .2f\n', C_exp(:,:,k)');
end

fclose(file_id);