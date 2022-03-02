classdef RotationTests < matlab.unittest.TestCase
    % RotationTests is a UnitTesting class for verifying the Rotation Class
    % defined in Rotation.m
    %
    % Use: While in the same folder as RotationTests.m and Rotation.m, run
    % in the command window:
    % >> runtests()
    %
    % This should return a detailed spread on the current defined tests:
    %   axangle_2_rotm_case1()
    %   axangle_2_rotm_case2()
    %   axangle_2_rotm_case3()
    %   rotm_2_quaternion()
    %   rotm_2_euler_RPY()

    methods (Test)

        function axangle_rotm_case1( test_case )
            % Case 1: theta = 0
            % define givens
            theta = 0;
            w = [1; 0; 0];
            
            % convert to and from
            rotm = Rotation.axangle2rotm(w,theta);
            [w2, theta2] = Rotation.rotm2axangle(rotm);
            
            % check validity
            tolerance = 0.001;
            theta_check = abs(theta-theta2) <= tolerance;
            w_check = abs(w-w2)<=tolerance;
            
            test_case.verifyTrue(all([ theta_check; w_check ]) )
        end

        function axangle_rotm_case2( test_case )
            % Case 2: theta = pi
            % define givens
            theta = pi;
            w = [1; 0; 0];
            
            % convert to and from
            rotm = Rotation.axangle2rotm(w,theta);
            [w2, theta2] = Rotation.rotm2axangle(rotm);
            
            % check validity
            tolerance = 0.001;
            theta_check = abs(theta-theta2) <= tolerance;
            w_check = abs(w-w2)<=tolerance;
            
            test_case.verifyTrue(all([ theta_check; w_check ]) )
        end
        
        function axangle_rotm_case3( test_case )
            % Case 3: General case
            % define givens
            theta = deg2rad(30);
            w = [0; 0.866; 0.5];
            
            % convert to and from
            rotm = Rotation.axangle2rotm(w,theta);
            [w2, theta2] = Rotation.rotm2axangle(rotm);
            
            % check validity
            tolerance = 0.001;
            theta_check = abs(theta-theta2) <= tolerance;
            w_check = abs(w-w2)<=tolerance;
            
            test_case.verifyTrue(all([ theta_check; w_check ]) )
        end

        function rotm_2_quaternion( test_case )
            % genaral case for quaternion and rotm conversion
            % define givens
            theta = deg2rad(30);
            w = [0; 0.866; 0.5];
            
            % convert to and from
            rotm = Rotation.axangle2rotm(w,theta);
            q = Rotation.rotm2quaternion(rotm);
            rotm2 = Rotation.quaternion2rotm(q);
            
            % check validity
            tolerance = 0.001;
            rotm_check = abs(rotm-rotm2) <= tolerance;
            
            test_case.verifyTrue( all(reshape(rotm_check,1,9)) )
        end

        function rotm_2_euler_RPY( test_case )
            % Test conversion of rotm2RPY
            % define givens
            theta = deg2rad(30);
            w = [0; 0.866; 0.5];
            
            % Get Euler Angles
            rotm = Rotation.axangle2rotm(w,theta);
            Phi = Rotation.rotm2euler(rotm, 'RPY');
            
            % check validity
            Phi_act = [0.0644047, 0.4478289, 0.2810408]';
            tolerance = 0.001;
            rotm_check = abs(Phi-Phi_act) <= tolerance;
            
            test_case.verifyTrue( all(rotm_check) )
        end
    end
end