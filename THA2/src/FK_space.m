function T = FK_space(M,S,q,plot)
% FK_space returns the space-form forward kinematics of the robot
% manipulator, defined by the home position M and the screw axes S.
%
% Use:
% T = FK_space(M,S,q)
%   - M is the home position Transform
%   - S is a 6xn matrix of the Space-Form Screw Axes for each joint
%   - q is an n-element array of joint positions
%   - plot - if true than plot the robot (default false)
%
%   See also FK_body

arguments
    M (4,4)
    S (6,:)
    q (:,1)
    plot logical = false;
end

% for plotting, store the exponentials
if plot
    J_frames = zeros(4,4,width(S)+1);
end

T = eye(4);
for i=1:width(S)
    T_i = screwAxis2TMat(S(1:3,i), S(4:6,i), q(i));
    T = T*T_i;

    if plot
        J_frames(:,:,i) = T;
    end
end
T = T*M;

% plot if specified
if plot
    J_frames(:,:,end) = T;
    plot_fk(S, J_frames)
end

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot_fk(S, J_frames)
figure();
px = plot3([0 0.2], [0 0], [0 0],'r');
    hold on; grid on;
py = plot3([0 0], [0 0.2], [0 0],'g');
pz = plot3([0 0], [0 0], [0 0.2],'b');

% find joint positions
J_pos = zeros(3,width(S)+1);
for k = 1:width(S)
    % Screw frame
    T_k = J_frames(:,:,k);

    % Find last time there was a velocity vect in order to determine
    % position
    for j = k:-1:1
        v_j = S(4:6,j);
        if any(v_j)
            break;
        end
    end
    P_init = cross(S(1:3,j),S(4:6,j));
    
    % Finally position of joint
    P_j = T_k*[P_init; 1];
    J_pos(:,k) = P_j(1:3);

    plot3(P_j(1),P_j(2),P_j(3), 'ro', MarkerFaceColor='r')
end
J_pos(:,end) = J_frames(1:3,4,end);

pj = plot3(J_pos(1,end), J_pos(2,end), J_pos(3,end), 'ro', MarkerFaceColor='r');
pl = plot3(J_pos(1,:), J_pos(2,:), J_pos(3,:), 'k', 'LineWidth', 2.0);
    xlabel('x'), ylabel('y'), zlabel('z')
    title('Forward Kinematics')
    legend([pl pj px py pz], ...
        {'$Link_i$', '$Joint_i$','$\hat{x}$', '$\hat{y}$','$\hat{z}$'}, ...
        'Interpreter', 'latex', 'FontSize',15)
    hold off

xl = xlim;
yl = ylim;
zl = zlim;
lims = [min([xl,yl,zl]) max([xl,yl,zl])];
    xlim(lims), ylim(lims), zlim([0 lims(2)])
    
end


