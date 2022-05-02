function [X] = ax_xb_noisy(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config)
%  %  ax_xb calculates the missing Rx and qx 
%  (rotation and translation between the camera and robot manipulator)
%   using the quaternion approach  with noisy data
%
% Use:
% [X] = ax_xb_noisy(q_Robot_config, q_camera_config)
% - q_Robot_config is a nx4 matrix of different robot configurations Ei
% - q_camera_config is a nx4 matrix of different sensor configurations Si
%
% Output:
%  [X] = A number 3X3 rotation matricies depending on the number of sensor and
%  and robot configurations and the last column a number of 3X1 vectors representing
%  translation for its corresponding 3X3 rotation matrix 
%  
%
%  
arguments
    q_Robot_config (:,4)       
    q_camera_config (:,4)     
    t_Robot_config (:,3)  
    t_camera_config (:,3)  
end


% q_Robot_config=[
%     -0.321, 0.087, 0.682, 0.651
%     -0.747, 0.431, -0.061, 0.502
%     -0.361, 0.310, 0.045, 0.878
%     -0.676, 0.414, -0.038, 0.608
%     -0.772, 0.256, 0.204, 0.545
%     -0.070, 0.530, 0.113, 0.838
%     -0.069, 0.582, 0.113, 0.803
%     -0.100, 0.470, 0.167, 0.861
%     -0.129, 0.463, 0.221, 0.849
%     0.137, 0.433, 0.165, 0.876];
% 
% 
% q_camera_config=[
%       0.608, -0.319, 0.723, -0.077
%       0.205, 0.383, -0.730, 0.527
%       -0.064, -0.309, 0.948, -0.034
%       -0.178, -0.379, 0.803, -0.424
%       -0.104, 0.313, -0.778, 0.534
%       -0.078, -0.536, 0.809, 0.226
%       -0.096, -0.587, 0.775, 0.213
%       -0.007, -0.502, 0.840, 0.205
%       0.046, -0.513, 0.839, 0.173
%       0.003, -0.465, 0.773, 0.431];
% 

% t_Robot_config=[
%       0.261, -0.369, 0.488
%       0.100, 0.446, 0.506
%       -0.099, -0.166, 0.677
%       0.050, 0.292, 0.299
%       -0.172, 0.233, 0.287
%       0.379, -0.057, 0.557
%       0.374, -0.058, 0.347
%       0.263, -0.040, 0.398
%       0.304, 0.008, 0.380
%       0.254, -0.132, 0.391];
%   
%   t_camera_config=[
%       0.252, 0.112, 0.577
%       -0.282, 0.037, 0.723
%       -0.312, 0.191, 0.836
%       -0.076, -0.074, 0.595
%       0.188, 0.023, 0.727
%       0.048, 0.104, 0.549
%       -0.039, -0.007, 0.363
%       0.033, -0.019, 0.452
%       0.103, -0.016, 0.401
%       0.069, -0.013, 0.463];
%       



a = 0; b = 1;
rng(1)
r = a + (b-a).*rand(1,4);
e = .0001;
r = r/sum(r)*e;
r = r + [0 e/2 e/4 e/4];
q_Robot_config = q_Robot_config - r;

rng(2)
r = a + (b-a).*rand(1,4);
r = r/sum(r)*e;
r = r + [0 e/2 e/4 e/4];
q_camera_config = q_camera_config - r;

rng(3)
r = a + (b-a).*rand(1,3);
r = r*0.0003;
t_Robot_config = t_Robot_config - r;

rng(4)
r = a + (b-a).*rand(1,3);
r = r*0.0005;
t_camera_config = t_camera_config - r;
  

N = length(q_Robot_config);
  
 
  
  for j = 1:1:N-1
      
      qa1 = Rotation.quaternion2rotm((q_Robot_config(j,:))'); % turining Ei quat to rotmat
      qa2 = Rotation.quaternion2rotm((q_Robot_config(j+1,:))'); % turining Ei+1 quat to rotmat
      qa3 = Rotation.rotm2quaternion(inv(qa2)*(qa1)); % calculating A and then turing it to a quaternion 
      
      qb1 = Rotation.quaternion2rotm((q_camera_config(j,:))'); % turining S quat to rotmat
      qb2 = Rotation.quaternion2rotm((q_camera_config(j+1,:))'); % turining S quat to rotmat
      qb3 = Rotation.rotm2quaternion(qb2*inv(qb1)); % calculating B and then turing it to a quaternion 
      
      
      Sa = qa3(1);
      Sb = qb3(1);
      Va = qa3(2:4);
      Vb = qb3(2:4);
      
      I = eye(3);
      
     A = [ Sa- Sb, -(Va-Vb)'; (Va-Vb), ((Sa- Sb)*I + Rotation.skewify(Va+ Vb))];
      
      [U,S,V] = svd(A); % svd 
      
      transV = V';
      
    for  z = 1:1:4
     unit_quat(j,z) =  transV(z,4)'; % getting the fourth column of the V transpose
    end 
      
      
  end 
  d = 0;
  % turning all the quaternion from the forth column of V transpose to rotm 
  for z = 1:1:N-1
    matx = Rotation.quaternion2rotm((unit_quat(z,:))');
   for r = 1:1:3
        for b = 1:1:3
            X(r+d,b) = matx(r,b);
            
        end
       
   end    
  d = d +3;
  end  
  
   f = 0;
  for g = 1:1:((length(X)/3))
      
      qa1 = Rotation.quaternion2rotm((q_Robot_config(g,:))'); % turining Ei quat to rotmat
      qa2 = Rotation.quaternion2rotm((q_Robot_config(g+1,:))'); % turining Ei+1 quat to rotmat
      qa3 = (inv(qa2)*(qa1)); % calculating A
      
      qb1 = Rotation.quaternion2rotm((q_camera_config(g,:))'); % turining S quat to rotmat
      qb2 = Rotation.quaternion2rotm((q_camera_config(g+1,:))'); % turining S quat to rotmat
      qb3 = (qb2*inv(qb1)); % calculating B
      
      tempmat = X(1+f:3+f,1:3); % Rx matrix
      pa = (t_Robot_config(g,:))' ; % given translation pA
      pb = (t_camera_config(g,:))' ; % given translation pB
      Amat = qa3 - eye(3); % A matrix for least square method 
      bmat = tempmat*pb - pa; % b matrix for least square metho
      z = inv(Amat'*Amat)*Amat'*bmat; % x vector based on the least square method 
      
      for y = 1:1:3
      qx(g,y) = z(y);
      end
      
      f = f + 3 ;
  end
  count = 1;    
  for y = 1:1:length(X)/3
      for z = 1:1:3
       X(count,4) = qx(y,z); % adding the translation vector to the fourth column 
       count = count +1;
      end
   end
      
end


