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
    %   quaternion_2_rotm()
    %   rotm_2_axangle()
    %   rotm_2_quaternion()
    %   rotm_2_euler_ZYZ()
    %   rotm_2_euler_RPY()

    properties
        tolerance = 0.001;
    end

    methods (Test)

        function axangle_2_rotm_case1( test_case )
            % Case 1: theta = 0
            % define givens
            theta = 0;
            w = [1 0 0];
            rotm_act = eye(3);

            % calc rotm
            rotm = Rotation.axangle2rotm(w,theta);
            
            % check validity
            check = abs(rotm_act - rotm) < test_case.tolerance;
            check = reshape(check, 1, 9);
            
            test_case.verifyTrue(all(check))
        end

        function axangle_2_rotm_case2( test_case )
            % Case 2: theta = pi
            % define givens
            theta = pi;
            w = [1; 0; 0];
            rotm_act = -eye(3); rotm_act(1,1) = 1;

            % calc rotm
            rotm = Rotation.axangle2rotm(w,theta);
            
            % check validity
            check = abs(rotm_act - rotm) < test_case.tolerance;
            check = reshape(check, 1, 9);
            
            test_case.verifyTrue(all(check))
        end
        
        function axangle_2_rotm_case3( test_case )
            % Case 3: General case
            % define givens
            theta = deg2rad(30);
            w = [0; 0.866; 0.5];
            rotm_act = [  0.866 -0.250 0.433;
                          0.250  0.967 0.058;
                         -0.433  0.058 0.899 ];

            % calc rotm
            rotm = Rotation.axangle2rotm(w,theta);
            
            % check validity
            check = abs(rotm_act - rotm) < test_case.tolerance;
            check = reshape(check, 1, 9);
            
            test_case.verifyTrue(all(check))
        end

        function quaternion_2_rotm( test_case )
            % genaral case for quaternion and rotm conversion
            % define givens
            rotm = [  0.866 -0.250 0.433;
                      0.250  0.967 0.058;
                     -0.433  0.058 0.899 ];
            q = [0.9659 0 0.2242 0.1294]';

            % calc quaternion
            rotm_calc = Rotation.quaternion2rotm(q);
            
            % check validity
            check = abs(rotm - rotm_calc) < test_case.tolerance;
            check = reshape(check, 1, 9);
            
            test_case.verifyTrue(all(check))
        end
        
        function rotm_2_axangle( test_case )
            % general case for rotm2axangle conversion
            % define givens
            theta = deg2rad(30);
            w = [0; 0.866; 0.5];
            axangle_act = [theta; w];
            rotm_act = [  0.866 -0.250 0.433;
                          0.250  0.967 0.058;
                         -0.433  0.058 0.899 ];

            % calc rotm
            [w_calc, th_calc] = Rotation.rotm2axangle(rotm_act);
            axangle = [th_calc; w_calc];

            % check validity
            check = abs(axangle_act - axangle) < test_case.tolerance;
            
            test_case.verifyTrue(all(check))
        end

        function rotm_2_quaternion( test_case )
            % genaral case for quaternion and rotm conversion
            % define givens
            rotm = [  0.866 -0.250 0.433;
                      0.250  0.967 0.058;
                     -0.433  0.058 0.899 ];
            q = [0.9659 0 0.2242 0.1294]';

            % calc quaternion
            q_calc = Rotation.rotm2quaternion(rotm);
            
            % check validity
            check = abs(q - q_calc) <= test_case.tolerance;
            
            test_case.verifyTrue(all(check))
        end

        function rotm_2_euler_RPY( test_case )
            % Test conversion of rotm2RPY
            % define givens
            rotm = [  0.866 -0.250 0.433;
                      0.250  0.967 0.058;
                     -0.433  0.058 0.899 ];
            RPY = [0.2810 0.4478 0.0644];

            % Get Euler Angles
            RPY_calc = Rotation.rotm2euler(rotm, 'RPY');
            
            % check validity
            check = abs(RPY - RPY_calc) <= test_case.tolerance;
            
            test_case.verifyTrue(all(check))
        end

         function rotm_2_euler_ZYZ( test_case )
            % Test conversion of rotm2RPY
            % define givens
            rotm = [  0.866 -0.250 0.433;
                      0.250  0.967 0.058;
                     -0.433  0.058 0.899 ];
            ZYZ = [-3.0084 -0.4523 -3.0084];

            % Get Euler Angles
            ZYZ_calc = Rotation.rotm2euler(rotm, 'ZYZ');
            
            % check validity
            check = abs(ZYZ - ZYZ_calc) <= test_case.tolerance;
            
            test_case.verifyTrue(all(check))
        end
    end
end