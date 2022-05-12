function rotm = axangle2rotm(axis, angle)
    % axangle2rotm Converts axis-angle representation to a 3x3
    % rotm using Rodrigues' formula
    %
    % Parameters:
    %   axis - axis of rotation as a 3x1 vector
    %   angle - angle of rotation in radians
    %
    % Returns:
    %   rotm - 3x3 rotation matrix
    
    arguments
        axis (3,1) double
        angle double
    end
    
    % check if unit axis
    tolerance = 0.001;
    if abs(norm(axis)-1) > tolerance
        fprintf('Warning: in axangle2rotm(axis,angle), "axis"')
        fprintf(' is not a unit vector. It was converted ')
        fprintf('automatically. Please verify that this is fine.\n')
        axis = axis/norm(axis);
    end

    % Rodrigues' formula
    w_skew = skewify(axis);
    rotm = eye(3) + w_skew*sin(angle) + w_skew^2*(1-cos(angle));
end