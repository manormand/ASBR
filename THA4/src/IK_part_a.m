function [q, linear_error, condition, isotropy] = IK_part_a(M,S,q0,J_limits, d_tool, p_goal)

h = 0.1;
tol = 0.003;
distance = @(T) T(1:3,4);
linearError = @(q,i) norm( distance(FK_space(M,S,q(:,i))) - p_goal ); 

q = [q0];
i = 1;
linear_error = linearError(q,1);
while linear_error(i) > tol && i < 1000
    if linear_error(i) > 1
        dq = farDq(M,S,q(:,i),d_tool,p_goal);
    else
        dq = closeDq(M,S,q(:,i),d_tool,p_goal);
    end

    q(:, i+1) = q(:,i) + (dq + jointLimitComp(q(:,i),J_limits))*h;
    
    q(:,i+1) = jointLimits(q(:,i+1), J_limits);

    linear_error(i+1) = linearError(q,i+1);
    i = i + 1;
end

end


function q = jointLimits(q,J_limits)
q = max(q, J_limits(:,1));
q = min(q, J_limits(:,2));
end

function dq = farDq(M,S,q,d_tool,p)
% calc newton rapson

Tsd = eye(4); Tsd(1:3,4) = p + d_tool*[0 0 1]';
Tbd = FK_space(M,S,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;
Vb = [ Vb(1:3) ; 0 ; 0 ; 0 ];

dq = J_space(S,q)\Vb;

end

function dq = closeDq(M,S,q, d_tool, p)
Tsd = FK_space(M,S,q);
T_tool = eye(4); T_tool(3,4) = d_tool;
Tsd = Tsd*T_tool;
x_tool = Tsd(1:3,4);

delta = [x_tool-p; 0; 0; 0];

n = height(q);
A = zeros(n,6);
for i = 1:n
    alpha = i*2*pi/n;
    beta  = i*2*pi/n;
    A(i,:) = [cos(alpha)*cos(beta) cos(alpha)*sin(beta) sin(alpha) 0 0 0];
end

b = ones(n,1) - A*delta;

dx = A\b;

dq = J_space(S,q)\dx;
end

function dq = jointLimitComp(q, J_limits)

lower = J_limits(:,1);
upper = J_limits(:,2);

A = [eye(height(q)); -eye(height(q))];
b = [upper-q; q-lower];
dq = A\b;
end