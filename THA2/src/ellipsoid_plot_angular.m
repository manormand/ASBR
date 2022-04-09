function ellipang = ellipsoid_plot_angular(S, q)
% ellipsoid_plot_angular calculates the jacobian, then calculates the 
% eigenvalues and eigenvectors, and uses the eigenvectors as the direction
% of the principle aes of the angular ellipsoid and uses the the sqrt of 
% the eigenvalues as the lengths of the principle semi-axes of the angular
% ellipsoid 
%
% Use:
% ellipang = ellipsoid_plot_angular(S, q)
%   - S is a 6xn matrix of the Space-Form Screw Axes for each join
%   - q is a n-element array of the joint positions
%
%   See also J_space

arguments
    S (6,:)        % Skew Axes in Space Frame
    q (1,:)        % Initial joint positions
end

% Get the jacobian and its transpose
j = J_space(S,q);
j = j(1:3,1:7);
jtrans = j.';
A = j*jtrans;

% calculate the eigenvector and eigenvalues for the angular case
[V,D] = eig(A);
[d,ind] = sort(diag(D));
d = sqrt(d);
V1 = V(1:3,1);
V2 = V(1:3,2);
V3 = V(1:3,3);
V1 = V1/norm(V1)*d(1);
V2 = V2/norm(V2)*d(2);
V3 = V3/norm(V3)*d(3);
V = [V1,V2,V3];
[w_calc, th_calc] = Rotation.rotm2axangle(V);
D = rad2deg(th_calc);

%plot 
figure()
[X,Y,Z] = ellipsoid(0,0,0,d(1),d(2),d(3));
h = surf(X,Y,Z);
hold on
rotate(h, w_calc, D);
axis equal
hold off

ellipang = 'ellipsoid_plot_angular';
end



