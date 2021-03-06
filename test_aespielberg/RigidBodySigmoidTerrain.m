classdef RigidBodySigmoidTerrain < RigidBodyHeightMapTerrain
%  This class provides an implementation of RigidBodyTerrain with a step
%  from z = 0.  Step is oriented in the x-axis.
%  Uses a linear step up/down.
%  Currently returns normal with NON-UNIT value. This is because
%  phi(q) != distance, and the normal is used with forward kinematics to
%  calculate dphi/dq
  
  methods 
    function obj = RigidBodySigmoidTerrain(step_height,step_x,smoothing_width)
%       x = -10:.01:10;
s = 10;
      x = [-s:0.1:s];
      y = [-s:0.1:s];
      
      [X,Y] = ndgrid(x,y);
      
      Z = zeros(length(x),length(y));
      for i = 1:1:length(x)
          for j = 1:1:length(y)
              Z(i, j) = sigmf(y(i), [smoothing_width, step_x]) * step_height;
          end
      end
      
%       Z = zeros(length(x),length(y));
%       t = (Y - step_x + smoothing_width)/(2*smoothing_width);
%       I_s = find(t > 0 & t < 1);
%       Z(t >= 1) = step_height;
%       Z(t >0 & t < 1) = step_height*t(I_s);
      
      obj@RigidBodyHeightMapTerrain(x,y,Z',eye(4));
      obj.c = hex2dec({'ee','cb','ad'})'/256;
        
      obj.height = step_height;
      obj.step_x = step_x;
      obj.smoothing = smoothing_width;
    end    
    
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      
     % z = zeros(1,n);
      %t = (xy(1,:) - obj.step_x + obj.smoothing)/(2*obj.smoothing);
      
%       z(t >= 1) = obj.height;
%       I_s = find(t > 0 & t < 1);
%       z(I_s) = obj.height*t(I_s);
     % normal = repmat([0;0;1],1,n);
      %normal(1,I_s) = -obj.height/(2*obj.smoothing);
      y_idx = 2;
      for i = 1:1:n
          z(i) = sigmf(xy(y_idx, i), [obj.smoothing, obj.step_x]) * obj.height;
        normal(:, i) = [0; sigmf(xy(y_idx, i), -[obj.smoothing, obj.step_x]) * obj.height; 1];
      end
    end
    
    function pts = getPoints(obj)
      mx = obj.step_x - obj.smoothing;
      px = obj.step_x + obj.smoothing;
      x = [-10 mx px 10 10 px mx -10];
      y = zeros(1,8);
      z = [0 0 obj.height obj.height -10*ones(1,4)];
      
%       x = [x x];
%       y = [y y];
%       z = [z -10*ones(1,8)];
      pts = obj.T(1:3,:)*[x;y;z;ones(1,8)];
    end
    
    function [x,y,z,c] = getPatchData(obj,x_axis,y_axis,view_axis)
      % output is compatible with patch for 2D viewing
      % still returns a 3xn, but the z-axis is constant (just meant for
      % depth ordering in the 2d figure)
      
      Rview = [x_axis, y_axis, view_axis]';
      valuecheck(svd(Rview),[1;1;1]);  % assert that it's orthonormal
      
      pts = getPoints(obj);
      
      % project it into 2D frame
      pts = Rview*pts;
      
      % keep only convex hull (todo: do better here)
      z = max(pts(3,:));
      
      % take it back out of view coordinates
      pts = Rview'*[pts(1:2,:); repmat(z,1,size(pts,2))];
      x = pts(1,:)';
      y = pts(2,:)';
      z = pts(3,:)';
      c = obj.c;
    end
  end
  
  properties
    height;
    step_x;
    smoothing;
    PTS;
  end
end