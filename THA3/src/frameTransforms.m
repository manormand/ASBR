function Fd = frameTransforms(df, for_D)
arguments
    df struct
    for_D {logical} = 1
end

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
