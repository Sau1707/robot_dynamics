function [ T_IE ] = jointToEndeffectorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  l32 = params.l32;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
    
  % Implement your solution here ...
   
  TI0 = [0, 1,  0,  0;
         0, 0,  1,  0;
         1, 0,  0,  l0;
         0, 0,  0,  1];


  T01 = [ cos(q(1)),  0,        sin(q(1)),      0;
          0,          1,        0,              l1;
          -sin(q(1)), 0,        cos(q(1)),      0;
          0,          0,        0,              1];
  
  
  


  T12 = [cos(q(2)), -sin(q(2)), 0,     0;
         sin(q(2)),  cos(q(2)), 0,     0;
              0,       0,       1,     0;
              0,       0,       0,     1];
  T23 = [cos(q(3)), -sin(q(3)), 0,     l2;
         sin(q(3)),  cos(q(3)), 0,     0;
              0,       0,       1,     0;
              0,       0,       0,     1];
  
  
  
  


  T3E = [1, 0,  0,  l31+l32;
         0, 1,  0,  0;
         0, 0,  1,  0;
         0, 0,  0,  1];


  T_IE = TI0*T01*T12*T23*T3E; 
  
end