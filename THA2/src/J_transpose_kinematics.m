function q_des = J_transpose_kinematics(M,B,q,Tsd, tol_w, tol_v)

arguments
    M (4,4)        % Home postition of end effector
    B (6,:)        % Skew Axes in Body Frame
    q (:,1)        % Initial joint positions
    Tsd (4,4)      % Desired final pose
    tol_w = 0.001  % orientation tolerance in rad
    tol_v = 0.0001 % distance tolerance in m
end

