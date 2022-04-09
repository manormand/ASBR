function sing = singularity(S, q)
% singularity calculates the jacobian, then calculates the rank and the
% determinant to check if the current configuration is in a singularity
% position
% 
%
% Use:
% sing = singularity(S, q)
%   - S is a 6xn matrix of the Space-Form Screw Axes for each join
%   - q is a n-element array of the joint positions
%
%   See also J_space

arguments
    S (6,:)        % Skew Axes in Space Frame
    q (1,:)        % Initial joint positions
end

% Get the jacobian and its inverse
j = J_space(S,q);
jtrans = j.';
A = j*jtrans;
jdet = det(A);
jrank = rank(A);

% use the determinant and rank to check if the A matrix is singular
if (jdet == 0 || jrank ~= 6)
    sing = true;
else 
    sing =  false;
end



