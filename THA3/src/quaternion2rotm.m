function rotm = quaternion2rotm(q)
% quaternion2rotm converts a 4x1 quaternion [w,x,y,z] to 
% rotation matrix representation
%
% Parameters:
%   q - input 4x1 quaternion representation of a rotation
%
% Returns:
%   rotm - 3x3 rotation matrix

arguments
    q (4,1) double
end

% check if unit quaternion
tolerance = 0.001;
if abs(norm(q)-1) > tolerance
    fprintf('Warning: in quaternion2rotm(q), "q"')
    fprintf(' is not a unit vector. It was converted ')
    fprintf('automatically. Please verify that this is fine.\n')
    q = axis/norm(q);
end

rotm = zeros(3);

% calc diagonal of rotm
for i=1:3
    signs_vect = -1*ones(4,1);
    signs_vect([1 i+1]) = 1;

    rotm(i,i) = sum(q.^2 .* signs_vect);
end

% calc rest of rotm
for i=1:3
    for j=1:3
        % skip if on main diagonal
        if i==j
            continue
        end
        % 3rd index
        k = 6 - i - j;
        
        % sign of second term
        if (j==k+1 ||j==k-2), sost=1; else, sost=-1; end

        rotm(i,j) = 2*(q(i+1)*q(j+1) + sost*q(1)*q(k+1));
    end
end
end