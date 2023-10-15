function [ J_3C_3 ] = point3ToCameraGeometricJacobian(q, params)
  % Inputs:
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  % Output:
  %     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
  %     frame {3}
  
  theta = params.theta;
  l31 = params.l31;
  l4 = params.l4;
  l5 = params.l5;


  q1 = q(1);
  q2 = q(2);
  q3 = q(3);




  C03 = [cos(theta-pi/2), 	-sin(theta-pi/2),   0;
        sin(theta-pi/2),  	cos(theta-pi/2), 	0;
        0,       		    0,       	        1, ]

  r03C = [l31;0;0] + C03*[l5;l4;0]

  C13 = [cos(q2+q3), -sin(q2+q3), 0;
         sin(q2+q3), sin(q2+q3), 0;
         0,          0,          1]
  n1 = C13'*[0,0,1];
  n2 = [1,0,0];
  n3 = [1,0,0];

  % Implement your solution here...
  J_3C_3 = zeros(6,3);
  J_3C_3(1:3,:) = [cross(n1,r03C),cross(n2,r03C),cross(n3,r03C)];


  

end