function J_vol = J_ellipsoid_volume(S,B, q)
%  J_ellipsoid_volume calculates the volume of the manipulabilty 
% ellipsoid 
%
% Use:
% J_ellipsoid_volume
%   - S is a 6xn matrix of the Space-Form Screw Axes for each join
%   - B is a 6xn matrix of the Body-Form Screw Axes for each join
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

% calculate the volume
vol_a = sqrt(det(A_a));
vol_l = sqrt(det(A_l));

J_vol = [vol_a vol_l];

end

