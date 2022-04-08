function q_des = J_transpose_kinematics(M,B,q,Tsd, tol_w, tol_v)

arguments
    M (4,4)        % Home postition of end effector
    B (6,:)        % Skew Axes in Body Frame
    q (:,1)        % Initial joint positions
    Tsd (4,4)      % Desired final pose
    tol_w = 0.001  % orientation tolerance in rad
    tol_v = 0.0001 % distance tolerance in m
end

outside_tolerance = @(Vb) norm(Vb(1:3)) > tol_w || norm(Vb(4:6)) > tol_v;


Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
e = S*th;

% alpha = [e, J*J'*e] * pinv([J*J'*e, J*J'*e]);
K = 0.1*eye(6);

while outside_tolerance(e)
    q = q + J_body(B,q)'*K*e;

    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    e = S*th;

end

q_des = q;