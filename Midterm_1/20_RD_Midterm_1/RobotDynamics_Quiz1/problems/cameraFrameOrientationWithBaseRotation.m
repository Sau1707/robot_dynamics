function [ C_IC ] = cameraFrameOrientationWithBaseRotation(qW, qX, qY, qZ, q, params) 
% Input: 
%   qW, qX, qY, qZ: components of the quaternion q_I0 = [qW, qX, qY, qZ],
%                   which describes the orientation from the 0 frame to the
%                   inertial frame
%   q: a 3x1 vector of generalized coordinates
%   params: a struct of parameters
% Output:
%   C_IC: rotation matrix describing the camera frame orientation
%         with respect to the inertial frame when the base orientation is
%         not fixed. 
  
  
  q_I0 = [qW; qX; qY; qZ];
  
  % implement your solution here ...
  C_IC = [ ];
   
  
end