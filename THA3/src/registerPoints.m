function T = registerPoints(a, b)
% registerPoints perform point set registration algorithm
%   This also assumes that a and b are the same size
% T = registerPoints(a,b)
%   - a is the fixed static scene set
%   - b is the moving model set
%   - T is the optimal transform between a and b

N = height(a);

%% compute bars
a_bar = 0;
b_bar = 0;

for i = 1:N
    a_bar = a_bar + a(i,:);
    b_bar = b_bar + b(i,:);
end

a_bar = a_bar / N;
b_bar = b_bar / N;

%% compute tildas
a_tilda = a - repmat(a_bar, [N 1]);
b_tilda = b - repmat(b_bar, [N 1]);

%% find R using SVD Method

H = zeros(3);

for k = 1:N
    for i = 1:3
        for j = 1:3
            H(i,j) = H(i,j) + a_tilda(k,i)*b_tilda(k,j);
        end
    end
end

[U,~,V] = svd(H);

R = V*U';

% what if R is reflection?
if det(R) > -1.001 && det(R) < -0.999
    V(:,3) = -V(:,3);

    R = V*U';
end


%% Find p
p = b_bar' - R*a_bar';

%% transform
T = [ R p; 0 0 0 1];