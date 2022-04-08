function J_iso = J_isotropy(S,B,q)
%  J_isotropy calculates the jacobian, then calculates the eigenvalues and
% used it to calculate the ratio of the longest abd shortest semi-axes of 
% the angular and linear manipulability ellipsoid 
%
% Use:
% J_iso = J_isotropy(S, q)
%   - S is a 6xn matrix of the Space-Form Screw Axes for each join
%   - q is a n-element array of the joint positions
%
%   See also J_space

arguments
    S (6,:)        % Skew Axes in Space Frame 
    B (6,:)        % Skew Axes in Body Frame
    q (1,:)        % Initial joint positions
end

% Get the jacobian and its transpose
js = J_space(S,q);
jb = J_body(B,q);
ja = js(1:3,1:7);
jl = jb(4:6,1:7);
jtransa = ja.';
jtransl = jl.';
A_a = ja*jtransa;
A_l = jl*jtransl;

% calculate the eigenvector and eigenvalues for the linear and angular case
[V_a,D_a] = eig(A_a);
[V_l,D_l] = eig(A_l);
[d_a,ind_a] = sort(diag(D_a));
[d_l,ind_l] = sort(diag(D_l));
d_a = sqrt(d_a);
d_l = sqrt(d_l);
%calculate the isotropy
if(min(d_a)~=0 && min(d_l)~=0 )
    J_iso = [max(d_a)/min(d_a),max(d_l)/min(d_l)];
else
    J_iso ='infinity (singular)';
end

