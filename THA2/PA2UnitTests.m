classdef PA2UnitTests < matlab.unittest.TestCase
    % PA2UnitTests Unit Tests for PA2 functions.
    %
    % PA2UnitTests Properties:
    %   tol - Tolerance of tests
    %
    % PA2UnitTests Methods:
    %   FK_space_UR5_6R - Test for FK_space
    %   FK_body_WAM_7R - Test for FK_body
    %   J_space_RRRP - Test for J_space
    %   J_body_4R - Test for J_body
    %   J_inv_kinematics_simple - Test for J_inverse_kinematics

    properties
        tol = 0.001; % Desired tolerance for tests
    end

    methods (Test)
        function FK_space_UR5_6R( test_case )
            % FK_space_UR5_6R Test for FK_space()
            %   Example 4.5 from Modern Robotics
            %
            %   See also FK_space

            % define robot params
            W1 = 0.109;
            W2 = 0.082;
            L1 = 0.425;
            L2 = 0.392;
            H1 = 0.089;
            H2 = 0.095;
            
            q = [0 -pi/2 0 0 pi/2 0];

            M = [   -1 0 0 L1+L2;
                     0 0 1 W1+W2;
                     0 1 0 H1-H2;
                     0 0 0     1];

            % Define Screw Axes
            w1 = [0 0 1]';
            w2 = [0 1 0]';
            w5 = [0 0 -1]';
            w3=w2;w4=w2;w6=w2;

            v1 = [0 0 0]';
            v2 = [-H1 0 0]';
            v3 = [-H1 0 L1]';
            v4 = [-H1 0 L1+L2]';
            v5 = [-W1 L1+L2 0]';
            v6 = [H2-H1 0 L1+L2]';

            S = [ w1 w2 w3 w4 w5 w6;
                  v1 v2 v3 v4 v5 v6];

            % generate T matrices
            T_given = [ 0 -1 0 0.095;
                        1  0 0 0.109;
                        0  0 1 0.988;
                        0  0 0     1];
            
            T_calc = FK_space(M,S,q);

            % check if equivalent
            verifyEqual(test_case, T_calc, T_given, ...
                        'AbsTol', test_case.tol)
        end

        function FK_body_WAM_7R( test_case )
            % FK_body_WAM_7R Test for FK_body()
            %   Example 4.7 from Modern Robotics
            % 
            %   See also FK_body

            % define robot params
            L1 = 0.550;
            L2 = 0.300;
            L3 = 0.060;
            W1 = 0.045;
            q = deg2rad([0 45 0 -45 0 -90 0]);

            M = eye(4); M(3,4) = L1+L2+L3;

            % Define Screw Axes
            w1 = [0 0 1]';
            w2 = [0 1 0]';
            w3=w1; w5=w1; w7=w1;
            w4=w2; w6=w2;

            v1 = [0 0 0]';
            v3=v1; v5=v1; v7=v1;
            v2 = [L1+L2+L3 0 0]';
            v4 = [L2+L3 0 W1]';
            v6 = [L3 0 0]';

            B = [   w1 w2 w3 w4 w5 w6 w7;
                    v1 v2 v3 v4 v5 v6 v7];

            % generate T matrices
            T_given = [ 0 0 -1 0.3157;
                        0 1  0      0;
                        1 0  0 0.6571;
                        0 0  0      1];
            
            T_calc = FK_body(M,B,q);

            % check if equivalent
            verifyEqual(test_case, T_calc, T_given, ...
                        'AbsTol', test_case.tol)
        end

        function J_space_RRRP( test_case )
            % J_space_RRRP Test for J_space
            %   Example 5.2 from Modern Robotics
            %
            %   See also J_space
            
            % Define variables arbitrarily
            L1 = 2.0;
            L2 = 2.5;
            q = deg2rad([30 45 60 0]);
            q(4) = 1.5;   % in m

            q1=q(1); q2=q(2);

            % Define Screw Axes
            w1 = [0 0 1]';
            v1 = [0 0 0]';

            w2 = [0 0 1]';
            v2 = [0 -L1 0]';

            w3 = [0 0 1]';
            v3 = [0 -L1-L2 0]';

            w4 = [0 0 0]';
            v4 = [0 0 1]';

            S = [ w1 w2 w3 w4;
                  v1 v2 v3 v4];

            % Generate jacobians
            J_given = [ 0           0                         0 0;
                        0           0                         0 0;
                        1           1                         1 0;
                        0  L1*sin(q1)  L1*sin(q1)+L2*sin(q1+q2) 0;
                        0 -L1*cos(q1) -L1*cos(q1)-L2*cos(q1+q2) 0;
                        0           0                         0 1];

            J_calc = J_space(S,q);

            % check if equivalent
            verifyEqual(test_case, J_calc, J_given, ...
                        "AbsTol", test_case.tol)
        end

        function J_body_4R( test_case )
            % J_body_4R Test for J_body
            %   From MR toolbox
            %  <a href="https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/JacobianBody.m">Github Link</a>
            %
            %   See also J_body
            
            Blist = [[0; 0; 1;   0; 0.2; 0.2], ...
                     [1; 0; 0;   2;   0;   3], ...
                     [0; 1; 0;   0;   2;   1], ...
                     [1; 0; 0; 0.2; 0.3; 0.4]];
            thetalist = [0.2; 1.1; 0.1; 1.2];
            
            % Generate Jacobians
            J_given = [-0.0453    0.9950         0    1.0000
                        0.7436    0.0930    0.3624         0
                       -0.6671    0.0362   -0.9320         0
                        2.3259    1.6681    0.5641    0.2000
                       -1.4432    2.9456    1.4331    0.3000
                       -2.0664    1.8288   -1.5887    0.4000];

            J_calc = J_body(Blist,thetalist);

            % check if equivalent
            verifyEqual(test_case, J_calc, J_given, ...
                        "AbsTol", test_case.tol)
        end

        function J_inv_kinematics_simple( test_case )
            % simple test case for inverse kinematics
            %   Example from Lecture W8-1
            %
            %   See also J_inverse_kinematics
            M = eye(4); M(1,4) = 2;
            B = [[0 0 1 0 2 0]' [0 0 1 0 1 0]'];
            
            Tsd = [-0.5 -0.866 0 0.366;
                    0.866 -0.5 0 1.366;
                    0 0 1 0;
                    0 0 0 1];
            
            q = J_inverse_kinematics(M,B,[0,pi/6]',Tsd);
            
            T_calc = FK_body(M,B,q);

            % check if equivalent
            verifyEqual(test_case, T_calc, Tsd, 'AbsTol', test_case.tol)
        end
    end
end