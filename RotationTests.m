classdef RotationTests < matlab.unittest.TestCase

    methods (Test)

        function case1_axangle_rotm( test_case )
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

        function case2_axangle_rotm( test_case )
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
        
        function case3_axangle_rotm( test_case )
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
    end
end