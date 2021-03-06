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
    max_iter int32 = 200     % Maximum iterations allowed
    frame_rate = 30
end

max_motor_speed = 2*ones(size(q));

% load robot
lbr = importrobot('iiwa7.urdf'); 
lbr.DataFormat = 'column';

% define tolerance function
outside_tolerance = @(Vb) norm(Vb(1:3)) > tol_w || norm(Vb(4:6)) > tol_v;

% get current matrix transforms
Tbd = FK_body(M,B,q)\Tsd;
[S, th] = tMat2ScrewAxis(Tbd);
Vb = S*th;

%% Setup Figure
figure()
set(gcf, 'position', [0 0 1920 1080])

% robot subplot
axr = subplot(5,2, 1:2:10 ); % this line makes a 5x2 grid and populates one side
plot3(Tsd(1,4),Tsd(2,4),Tsd(3,4), 'ro', 'MarkerFaceColor','r', 'MarkerSize',6)
hold on;
show(lbr,q);
    title('Inverse Kinematics', FontSize=14)
    xlabel('x'),ylabel('y'),zlabel('z')
    grid on, axis equal;
    axis([-0.5 1.0 -0.75 0.75 0, 1.5])

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

top_txt  = sprintf("            Linear | Angular\n--------------------------------------");
cond_txt = sprintf("Condition: %.3f | %2.3f", cond(1), cond(2));
iso_txt  = sprintf("Isotropy:  %.3f | %2.3f", iso(1), iso(2));

textr = text([0 0 0],[1.1, 0.7, 0.4],{top_txt, cond_txt, iso_txt}, ...
                'FontSize',20, 'FontName','FixedWidth', 'Interpreter','none');
    xlim([-0.01 0.3]), ylim([0 1.5])
    xticks([]), yticks([])
    set(gca, 'Box', 'on')

%% Setup animation
avi_path = 'avi/inverse_kin_ani.avi';
inv_ani = VideoWriter(avi_path);
inv_ani.FrameRate = frame_rate;
dt = 1/frame_rate;
open(inv_ani)

frame = getframe(gcf);
writeVideo(inv_ani,frame);

% loading thing
subplot(axr)
r = 0.15; n = ceil(2/dt);
hold_x = r*ones(1,n);
hold_y = zeros(1,n);
hold_z = zeros(1,n);
hold_ani = plot3(hold_x, hold_y, hold_z, 'b', ...
                'LineWidth',3);

drawnow
% pause(0.1)
for i = 2:n
    th = i/n * (2*pi);
    hold_x(i:n) = r*cos(th);
    hold_y(i:n) = r*sin(th);
    set(hold_ani, 'XData', hold_x, ...
                        'YData', hold_y)
    drawnow
    
    frame = getframe(gcf);
    writeVideo(inv_ani,frame);
end

%% Spin
i = 0;
while outside_tolerance(Vb) && i < max_iter
    % iterate to next position
    dq = J_body(B,q)\Vb;
    dq = dt*min(dq, max_motor_speed);
    q = q + dq;

    % update twist
    Tbd = FK_body(M,B,q)\Tsd;
    [S, th] = tMat2ScrewAxis(Tbd);
    Vb = S*th;

    i = i+1;

    % animation stuff
    updateRobot(lbr, q, axr)
    updateEllipsoids(h_lin, h_ang, B, q)
    updateTexts(textr, M, B, q)

    % pause(dt)
    drawnow limitrate % dont draw, just wait til the end

    frame = getframe(gcf);
    writeVideo(inv_ani,frame);
end
drawnow % now show animation
close(inv_ani)

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function updateRobot(robot, q, axr)
% Update robot plot!

axes(axr) % makes the current subplot the robot one
show(robot,q, FastUpdate=true, PreservePlot=false);

end

function updateEllipsoids(h_lin, h_ang,B,q)
% Updates all elipsoids!
[dl,wl,thl] = ellipsoid_plot_linear(B,q,false);
[da,wa,tha] = ellipsoid_plot_angular(B,q,false);

[X,Y,Z] = ellipsoid(0,0,0,dl(1),dl(2),dl(3));
set(h_lin, 'XData', X);
set(h_lin, 'YData', Y);
set(h_lin, 'ZData', Z);

[X,Y,Z] = ellipsoid(0,0,0,da(1),da(2),da(3));
set(h_ang, 'XData', X);
set(h_ang, 'YData', Y);
set(h_ang, 'ZData', Z);


rotate(h_lin,wl,thl)
rotate(h_ang,wa,tha)

end

function updateTexts(textr, M, B, q)
% Update all the texts!
cond = J_condition(M,B,q);
iso  = J_isotropy(M,B,q);

cond_txt = sprintf("Condition:   %.3f | %2.3f", cond(1), cond(2));
iso_txt  = sprintf("Isotropy:    %.3f | %2.3f", iso(1), iso(2));

set(textr(2), 'String', cond_txt)
set(textr(3), 'String', iso_txt)
end