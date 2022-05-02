function Fd = frameTransforms(df, for_D)
% frameTransforms computes the sepcified frame transform
%   Using either D or A coordinates, iterate over every frame to find F,
%   the frame transformation matrix
arguments
    df struct           % dataframe from getdataset()
    for_D {logical} = 1 % True for D, false for A
end

% use D or A
if for_D
    d = df.calbody.d;
    D = df.calreadings.D;
else
    d = df.calbody.a;
    D = df.calreadings.A;
end

n_frames = size(D,3);

Fd = zeros(4,4,n_frames);
for i = 1:n_frames
    Fd(:,:,i) = registerPoints(d, D(:,:,i));
end
