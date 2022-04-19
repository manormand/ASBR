function [d,w,th,h] = ellipsoid_plot_linear(B, q, new_fig)
% ellipsoid_plot_angular calculates the jacobian, then calculates the 
% eigenvalues and eigenvectors, and uses the eigenvectors as the direction
% of the principle aes of the linear ellipsoid and uses the the sqrt of 
% the eigenvalues as the lengths of the principle semi-axes of the linear
% ellipsoid 
%
% Use:
% ellip_lin = ellipsoid_plot_linear(S, q)
%   - S is a 6xn matrix of the Space-Form Screw Axes for each join
%   - q is a n-element array of the joint positions
%
%   See also J_space

arguments
    B (6,:)        % Skew Axes in Body Frame
    q (1,:)        % Initial joint positions
    new_fig  logical = true
end

% Get the jacobian and its transpose
j = J_body(B,q);
j = j(4:6,1:7);
jtrans = j.';
A = j*jtrans;

% calculate the eigenvector and eigenvalues for the linear case
[V,D] = eig(A);
[d,~] = sort(diag(D));
d = sqrt(d);
V1 = V(1:3,1);
V2 = V(1:3,2);
V3 = V(1:3,3);
% V1 = V1/norm(V1)*d(1);
% V2 = V2/norm(V2)*d(2);
% V3 = V3/norm(V3)*d(3);
V1 = V1/norm(V1);
V2 = V2/norm(V2);
V3 = V3/norm(V3);
V = [V1,V2,V3];

[w, th] = rotm2axangle(V);
D = rad2deg(th);

% normalise to destroy compex numbers
for i=1:3
    w(i) = real(w(i));
    d(i) = real(d(i));
end
D = real(D);
th = real(th);

%plot 
if new_fig
%     figure()
    [X,Y,Z] = ellipsoid(0,0,0,d(1),d(2),d(3));
    h = surf(X,Y,Z);
    hold on
    rotate(h, w, D);
    h.EdgeColor = 'none';
    title('ellipsoid plot linear')
    axis equal
    hold off 
end

% ellip_lin = 'ellipsoid_plot_linear';
end

