classdef Rotation
    % Rotation  Summary of Rotation
    % Rotation class that takes in a user input and (currently) stores it
    % as axis-angle format.
    %
    % Rotation Properties:
    %   rotm - the rotation matrix representation in a 3x3 matrix
    %
    % Rotation Static Methods:
    %   rotm2axangle - convert 3x3 rotation matrix to axis-angle
    %   representation
    %   rotm2quaternion - convert 3x3 rotation matrix to quaternion
    %   representation
    %   rotm2euler - convert 3x3 rotation matrix to 3 angle representation
    %   of either ZYZ or RPY
    %   axangle2rotm - convert axis-angle representation to a 3x3 rotm
    %   quaternion2rotm - convert quaternion representation to a 3x3 rotm
    %
    % Rotation Methods:
    %   Rotation - constructor function
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        rotm = ones(3);
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
            %   axis - axis of rotation as a 1x3 vector
            %   angle - angle of rotation in radians
            arguments
                rotm (3,3) double
            end
            axis = rotm(:,1);
            angle = 1;
        end

        function q = rotm2quaternion(rotm)
            % rotm2quaternion Converts a given rotation matrix to
            % quaternion representation.
            %
            % Parameters:
            %   rotm - 3x3 rotation matrix representation
            %
            % Returns:
            %   q - quaternion representation
            arguments
                rotm (3,3) double
            end
            q = [1 rotm(1,:)];
        end

        function angles = rotm2euler(rotm, format)
            % rotm2euler Converts a given rotation matrix to
            % euler angle representation. The user can specify the format
            % meaning ZYZ, RPY
            %
            % Parameters:
            %   rotm - 3x3 rotation matrix representation
            %   format - choose between 'ZYZ' (default) and 'RPY'
            % Returns:
            %   angles - a 1x3 vector of desired angles according to
            %   specification
            arguments
                rotm (3,3) double
                format {mustBeText} = 'ZYZ'
            end
            
            switch(format)
                case 'ZYZ'
                    % ZYZ
                case 'RPY'
                    % RPY
                otherwise
                    error('Error in rotm2euler:\n"%s" is not a recognized format', ...
                        format)
            end
            angles = rotm(1,:);
        end

        function rotm = axangle2rotm(axis, angle)
            % axangle2rotm Converts axis-angle representation to aa 3x3
            % rotm
            %
            % Parameters:
            %   axis - axis of rotation as a 1x3 vector
            %   angle - angle of rotation in radians
            %
            % Returns:
            %   rotm - 3x3 rotation matrix
            arguments
                axis (1,3) double
                angle double
            end
            rotm = axis*angle;
        end

        function rotm = quaternion2rotm(q)
            % quaternion2rotm converts a quaternion to a rotation matrix
            % representation
            %
            % Parameters:
            %   q - input quaternion representation of a rotation
            %
            % Returns:
            %   rotm - 3x3 rotation matrix

            arguments
                q (1,4) double
            end

            rotm = eye(3);
            rotm(1,:) = q(2:end);
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