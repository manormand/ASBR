function [X] = ax_xb(q_Robot_config, q_camera_config)
%  ax_xb calculates the missing Rx using the quaternion approach 
%  
%
% Use:
% [X] = ax_xb(q_Robot_config, q_camera_config)
% - q_Robot_config is a nx4 matrix of different robot configurations Ei
% - q_camera_config is a nx4 matrix of different sensor configurations Si
%   
%

arguments
    q_Robot_config (:,4)       
    q_camera_config (:,4)        
        
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

  N = length(q_Robot_config);
  
 
  
  for j = 1:1:N-1
      
      qa1 = Rotation.quaternion2rotm((q_Robot_config(j,:))'); % turining Ei quat to rotmat
      qa2 = Rotation.quaternion2rotm((q_Robot_config(j+1,:))'); % turining Ei+1 quat to rotmat
      qa3 = Rotation.rotm2quaternion(inv(qa1)*qa2); % calculating E^-1*E and then turing it to a quaternion 
      
      qb1 = Rotation.quaternion2rotm((q_camera_config(j,:))'); % turining S quat to rotmat
      qb2 = Rotation.quaternion2rotm((q_camera_config(j+1,:))'); % turining S quat to rotmat
      qb3 = Rotation.rotm2quaternion(qb1*inv(qb2)); % calculating S^-1*S and then turing it to a quaternion 
      
      
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
      
end

