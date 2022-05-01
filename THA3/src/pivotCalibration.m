function [b_post, b_box] = pivotCalibration(df, for_EM)

arguments
    df struct
    for_EM {logical} = 1
end

if for_EM
    G = df.empivot.G;
    Fd = eye(4);
else
    G = df.optpivot.H;
    Fd = frameTransforms(df); Fd = inv(Fd(:,:,1));
end

N = height(G);
n_frames = size(G,3);

G0 = sum(G(:,:,1), 1)/N;
g = G(:,:,1) - repmat(G0, [N, 1]);

A = zeros(n_frames*3, 6);
b = zeros(n_frames*3, 1);

for k = 1:n_frames
    Fk = Fd * registerPoints(g, G(:,:,k));

    Rk = Fk(1:3,1:3);
    pk = Fk(1:3, 4);

    start = (k-1)*3 + 1;
    stop = k*3;

    A(start:stop,:) = [Rk -eye(3)];
    b(start:stop) = -pk;
end

x = A\b;

b_box = x(1:3);
b_post = x(4:6);