function [ J_IG_p ] = jointToPositionJacobian(q, params)
  % Inputs:  
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_IG_p: position Jacobian of gripper in inertia frame (3,3)
  
  % link lengths (meters)
  l01 = params.l01;
  l02 = params.l02;
  l03 = params.l03;
  l04 = params.l04;
  l11 = params.l11;
  l21 = params.l21;
  l22 = params.l22;
  % angle (radians)
  alpha = params.alpha;

  % Joint positions
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);  
  
  % Implement your solution here...
  
  % Note: Formula in the script, eq. 2.155, is only applicable 
  %     for rotating joints, which are on the arm. For the base that
  %     only translates, we can account for its component directly.
  %
  % We apply eq. 2.154: J_IG_I = J_IB_I + J_BG_I
  %
  % Since, B is not a rotating frame, J_BG_B = J_BG_I

end