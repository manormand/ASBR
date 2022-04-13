function inv_kin_animation(M, B, q, Tsd, tol_w, tol_v, max_iter, frame_rate)
% inv_kin_animation animate a robot from position a to b
%   This function uses the Newton Raphson Method to locate
%   the joint positions required for the desired position, Tsd
%
% Use:
% inv_kin_animation(M, B, q0, Tsd)
%   - M is the home position of the robot, represented by a 4x4 T-Mat
%   - B is the body frame screw axes in a 6xn matrix
%   - q is the initial guess configuration in a column vector
%   - Tsd is the desired final configuration
%
% Optional:
% J_inverse_kinematics(..., tol_w, tol_v, max_iter)
%   - tol_w is the orientation tolerance (default 0.001)
%   - tol_v is the linear tolerance (default 0.0001)
%   - max_iter is the maximum iterations allowed (default 100)
%
%   See also J_transpose_kinematics, redundancy_resolution

arguments
    M (4,4)        % Home postition of end effector
    B (6,:)        % Skew Axes in Body Frame
    q (:,1)        % Initial joint positions
    Tsd (4,4)      % Desired final pose
    tol_w double = 0.001   % orientation tolerance in rad
    tol_v double = 0.0001  % distance tolerance in m
    max_iter int32 = 250     % Maximum iterations allowed
    frame_rate = 30
end

% load robot
lbr = importrobot('iiwa7.urdf'); 
lbr.DataFormat = 'column';

% define tolerance function
outside_tolerance = @(Vb) norm(Vb(1:3)) > tol_w || norm(Vb(4:6)) > tol_v;

% get current matrix transforms
Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;

% Setup Figure
figure()
set(gcf, 'position', [100 400 800 515])

% robot subplot
axr = subplot(5,2, 1:2:10 ); % this line makes a 5x2 grid and populates one side
plot3(Tsd(1,4),Tsd(2,4),Tsd(3,4), 'ro'), hold on;
show(lbr,q);
    title('Inverse Kinematics', FontSize=14)
    xlabel('x'),ylabel('y'),zlabel('z')
    grid on, axis equal;
    axis([-0.6 0.6 -0.6 0.6 0, 1.2])

% Linear Ellipsoid subplot
subplot(5,2,[2 4]);    % populate top right
[~,~,~,h_lin] = ellipsoid_plot_linear(B,q);
    title('')
    xlabel('x'),ylabel('y'),zlabel('Linear Ellipsoid','FontSize',14)
    grid on, axis equal
    axis([-1 1 -1 1 -1 1])

% Linear Ellipsoid subplot
subplot(5,2,[6 8]); % populate middle right
[~,~,~,h_ang] = ellipsoid_plot_angular(B,q);
    title('')
    xlabel('x'),ylabel('y'),zlabel('Angular Ellipsoid','FontSize',14)
    grid on, axis equal
    axis([-2 2 -2 2 -2 2])

subplot(5,2,10)    % populate bottom right
cond = J_condition(M,B,q);
iso  = J_isotropy(M,B,q);

cond_txt = sprintf("Condition: %.3f ", cond);
iso_txt  = sprintf("Isotropy:  %.3f ", iso);

textr = text([0 0],[0.5 1],{iso_txt, cond_txt});
    xlim([-0.1 0.5]), ylim([0 1.5])
    xticks([]), yticks([])
    set(gca, 'Box', 'on')

% setup animation
inv_ani = VideoWriter('animation/Inverse_Kin_ani.avi');
inv_ani.FrameRate = frame_rate;
dt = 1/frame_rate;
open(inv_ani)

drawnow
pause(0.1)

frame = getframe(gcf);
writeVideo(inv_ani,frame);

i = 0;
while outside_tolerance(Vb) && i < max_iter
    % iterate to next position
    q = q + J_body(B,q)\Vb;

    % update twist
    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    Vb = S*th;

    i = i+1;

    % animation stuff
    updateAllPlots(lbr, M, B,q, axr, h_lin, h_ang, textr)

    pause(dt)
    drawnow limitrate % dont draw, just wait til the end

    frame = getframe(gcf);
    writeVideo(inv_ani,frame);
end
drawnow % now show animation
end

function updateAllPlots(robot, M, B, q, axr, h_linear, h_angular, textr)
% updates all figures/texts and makes life easy

% robot
axes(axr)
show(robot,q, FastUpdate=true, PreservePlot=false);

% ellipses
updateEllipsoid(h_linear, B, q, "line")
updateEllipsoid(h_angular, B, q, "angle")

% text
cond = J_condition(M,B,q);
iso  = J_isotropy(M,B,q);

cond_txt = sprintf("Condition: %.3f ", cond);
iso_txt  = sprintf("Isotropy:  %.3f ", iso);

set(textr(1), 'String', cond_txt)
set(textr(2), 'String', iso_txt)

end

function updateEllipsoid(h,B,q, type)
% Updates a single ellipsoid. Specify linear or angulare with 'type'
if type=="line"
    [d,w,th] = ellipsoid_plot_linear(B,q,false);
else
    [d,w,th] = ellipsoid_plot_angular(B,q,false);
end

[X,Y,Z] = ellipsoid(0,0,0,d(1),d(2),d(3));
set(h, 'XData', X);
set(h, 'YData', Y);
set(h, 'ZData', Z);

% Sometimes robot no work.... why? idk man
if th~=real(th)
    msg = ['ERROR: Singularity hit?' num2str(singularity(B,q))];
    disp(q)
    error(msg)
end
rotate(h,w,th)
end