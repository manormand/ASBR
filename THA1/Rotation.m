classdef Rotation
    % Rotation Class contains static methods for converting rotation
    % representations and (WIP) member methods for storing and manipulating
    % rotations.
    %
    % Rotation Properties:
    %   rotm - the rotation matrix representation in a 3x3 matrix
    %
    % Rotation Static Methods:
    %   rotm2axangle - convert 3x3 rotation matrix to axis-angle
    %   representation
    %   rotm2quaternion - convert 3x3 rotation matrix to 4x1 quaternion
    %       representation
    %   rotm2euler - convert 3x3 rotation matrix to 3 angle representation
    %       of either ZYZ or RPY
    %   axangle2rotm - convert axis-angle representation to a 3x3 rotm
    %   quaternion2rotm - convert 4x1 quaternion representation to a 
    %       3x3 rotm
    %   skewify - convert a 3 element array to skew matrix form
    %
    % Rotation Methods:
    %   Rotation - constructor function
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        rotm = ones(3);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    enumeration
        % euler angle formats
        ZYZ
        RPY
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
        function [axis,angle] = rotm2axangle(rotm)
            % rotm2axangle Converts a given rotation matrix to
            % axis angle representation.
            %
            % Parameters:
            %   rotm - 3x3 rotation matrix representation
            %
            % Returns:
            %   axis - axis of rotation as a 3x1 vector
            %   angle - angle of rotation in radians
            arguments
                rotm (3,3) double
            end
      
            tr_R = sum(diag(rotm)); % Trace of rotm, tr(R)
            tolerance = 0.001;

            if rotm==eye(3)
                % Case 1: Angle = 0
                angle = 0;
                axis = [1; 0; 0]; % axis is undefined. Default to x-axis.
            elseif abs(tr_R+1) <= tolerance
                % Case 2: Angle = pi
                angle = pi;

                % Make sure the denominator is nonzero
                if rotm(1,1) ~= -1
                    axis = (2*(1+rotm(1,1)))^-0.5 * (rotm(:,1) + [1;0;0]);
                elseif rotm(2,2) ~= -1
                    axis = (2*(1+rotm(2,2)))^-0.5 * (rotm(:,2) + [0;1;0]);
                else
                    axis = (2*(1+rotm(3,3)))^-0.5 * (rotm(:,3) + [0;0;1]);
                end
            else
                % Case 3: General case
                angle = acos( 0.5*(tr_R - 1) );
                w_skew = ( 1/(2*sin(angle)) ) * (rotm - rotm');
                axis = [w_skew(3,2); w_skew(1,3); w_skew(2,1)];
            end
        end

        function q = rotm2quaternion(rotm)
            % rotm2quaternion Converts a given rotation matrix to a 4x1
            % quaternion representation.
            %
            % Parameters:
            %   rotm - 3x3 rotation matrix representation
            %
            % Returns:
            %   q - 4x1 quaternion representation [w,z,y,x]
            arguments
                rotm (3,3) double
            end

            % Setup q and calc scalar val
            q = zeros(4,1);
            q(1) = 0.5*sqrt( sum(diag(rotm))+1 );
            
            % set indexes for vector calc
            idxs = [3 1 2;
                    2 3 1];

            % solve for vector components
            for i=1:3
                a = idxs(1,i);
                b = idxs(2,i);
                q(i+1) = 0.5*sign(rotm(a,b) - rotm(b,a)) * ...
                    sqrt( rotm(i,i) - rotm(b,b) - rotm(a,a) + 1 );
            end
        end

        function angles = rotm2euler(rotm, format)
            % rotm2euler Converts a given rotation matrix to
            % euler angle representation. The user can specify the format
            % of euler angles to "ZYZ" or "RPY"
            %
            % Parameters:
            %   rotm - 3x3 rotation matrix representation
            %   format - choose between 'ZYZ' (default) and 'RPY'
            % Returns:
            %   angles - a 1x3 vector of desired angles according to
            %   format specification
            arguments
                rotm (3,3) double
                format Rotation % locks format to enum
            end
            
            switch(format)
                case Rotation.ZYZ
                    angles = rotm2zyz(rotm);
                case Rotation.RPY
                    angles = rotm2rpy(rotm);
            end
        end

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
            w_skew = Rotation.skewify(axis);
            rotm = eye(3) + w_skew*sin(angle) + w_skew^2*(1-cos(angle));
        end

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

        function w_skew = skewify(w)
            % skewify converts a 3 element vector to a skew matrix
            %
            % Parameters:
            %   w - a 3 element vector
            %
            % Returns:
            %   w_skew - 3x3 skew matrix of w

            w_skew = [   0    -w(3)   w(2);
                       w(3)      0   -w(1);
                      -w(2)    w(1)     0];
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj = Rotation(varargin)
            % Rotation Constructer for Rotation class based on a user given
            % rotation representation
            % 
            % Valid uses:
            %   Rotation(rotm)
            %   Rotation(axis, angle)
            %   Rotation(quaternion)
            %
            % Subfunctions:
            %   axangle2rotm, quaternion2rotm
            obj.rotm = eye(3);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helper Functions
% These functions are used for Euler representation derivation.
%

%
function rpy = rotm2rpy(rotm)
% Algorithm from _Modern Robotics_ Appendix B.1.1
if rotm(3,1)==1
    alpha = 0;
    beta  = -pi/2;
    gamma = -atan2(rotm(1,2), rotm(2,2));
elseif rotm(3,1)==-1
    alpha = 0;
    beta  = pi/2;
    gamma = atan2(rotm(1,2), rotm(2,2));
else
    alpha = atan2(rotm(2,1), rotm(1,1));
    beta  = atan2(-rotm(3,1), sqrt(rotm(3,2)^2 + rotm(3,3)^2));
    gamma = atan2(rotm(3,2), rotm(3,3));
end

rpy = [alpha beta gamma];
end

%
function zyz = rotm2zyz(rotm)
% Algorithm from Lecture Notes W3-L1
alpha = atan2(rotm(2,3), rotm(1,3));
beta  = atan2(sqrt(rotm(1,3)^2 + rotm(2,3)^2), rotm(3,3));
gamma = atan2(rotm(3,2), -rotm(3,1));

% check if beta < 0
if rotm(1,3)~=(cos(alpha)*sin(beta))
    alpha = atan2(-rotm(2,3), -rotm(1,3));
    beta  = atan2(-sqrt(rotm(1,3)^2 + rotm(2,3)^2), rotm(3,3));
    gamma = atan2(-rotm(3,2), rotm(3,1));
end

zyz = [alpha beta gamma];
end