# Midterm 1 - Robot Dynamics


## MatLab Basics
```matlab
% Get joint angles
q0 = q(1);
q1 = q(2);

% Vector 
v = [1; 2; 3];

% Matrix
M = [1 2 3; 4 5 6; 7 8 9];
M = [1, 2, 3; 4, 5, 6; 7, 8, 9];
```
<br />

## Position vector and Rotation matrix
> Apply the rotation matrix to the vector
$$_Ar_{12} = \ _AC_B \cdot \ _Br_{12}$$
$$_Ar_{2G} = \ _AC_B \cdot \ _BC_D \cdot \ _Dr_{2G}$$

```matlab
r_12_A = [l_x; l_y; l_z];
```
> r_12_A: position vector from point 1 to point 2 in frame A


### No rotation
> The frame is just shifted
```matlab
C = eye(3);
```

### Axis flip:
> The new frame is flipped:
```matlab
C = [0 0 1;  % Old x-axis is new z-axis
     1 0 0;  % Old y-axis is new x-axis
     0 1 0]; % Old z-axis is new y-axis
```


### X-axis rotation
> Replace a with the angle of rotation.
```matlab
C = [1      0       0;
     0      cos(a)  −sin(a);
     0      sin(a)  cos(a)];
```

### Y-axis rotation
> Replace a with the angle of rotation.
```matlab
C = [cos(a) 0   sin(a);
     0      1        0;
     −sin(a) 0   cos(a)];
```

### Z-axis rotation
> Replace a with the angle of rotation.
```matlab
C = [cos(a)  −sin(a) 0;
     sin(a)  cos(a)  0;
     0       0    1];
```

<br />

## Transformation matrix

> Created by a position vector p and a rotation matrix C.
```matlab
r = [l_x; l_y; l_z];    % translation vector
C = eye(3);             % rotation matrix
T = [C r; 0 0 0 1];
```
> Final translation matrix is obtained by multiply all of them:

$$T_{1n} = T_{12} \cdot T_{23} \cdot ... \cdot T_{6n}$$


<br />

## Position Jacobian
Each element refer to the corresponding joint angle. (i.e. $q_0, \, q_1, \, \dots$)
$$ J_P = [n_0 \times r_{0(n + 1)} \quad n_1 \times r_{1(n + 1)} \quad ... \quad n_n \times r_{n(n + 1)}]$$

> Where $n$ is the normal vector of the joint and r is the vector from the origin to the next frame or end effector.

Geometric Jacobian can be simply added: 
$$_AJ_C = \ _AJ_B + \ _AJ_{BC}$$

### Translates
> Account for its component directly
```matlab
J  = [[0; 1; 0], zeros(3, 1), zeros(3, 1)]; % Translation in y-axis, on angle 0
```

### Rotating joints
```matlab
J = [zeros(3, 1), cross(n_1, r_1), cross(n_2, r_2)]; % Both vector in frame A
```

<br />

## Rotation Jacobian
$$ J_R = [n_0 \quad n_1 \quad ... \quad n_n]$$
> Where $n$ is the normal vector of the rotation.

```matlab
J = [zeros(3,1), n_1, n_2]; % n_0 is a translation therefore no normal vector
```

<br />

## Inverse Kinematics


<br />
<br />
<br />
<br />

# Quick access exercises solution


## quatMult

```matlab
function quat_AC = quatMult(quat_AB,quat_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  q = quat_AB;
  p = quat_BC;
  
  q_w = q(1); q_n = q(2:4);
  p_w = p(1); p_n = p(2:4);

  quat_AC = [q_w*p_w - q_n'*p_n;
             q_w*p_n + p_w*q_n + skewMatrix(q_n)*p_n];
end

function A = skewMatrix(q_n)
    A = [0, -q_n(3), q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end
```

## quatToRotMat

```matlab
function C = quatToRotMat(quat)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % Extract the scalar part.
  quat_w = quat(1);
  
  % Extract the vector part.
  quat_n = quat(2:4);
  
  % Map the unit quaternion to a rotation matrix.
  C = (2*quat_w^2-1)*eye(3) + 2.0*quat_w*skewMatrix(quat_n) + 2.0*(quat_n*quat_n');
end

function A = skewMatrix(q_n)
    A = [0, -q_n(3), q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end

```

## rotMatToQuat

```matlab
function quat = rotMatToQuat(C)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  quat = 0.5*[sqrt((1+trace(C)));
              sign(C(3,2)-C(2,3)) * sqrt(C(1,1) - C(2,2) - C(3,3) + 1);
              sign(C(1,3)-C(3,1)) * sqrt(C(2,2) - C(3,3) - C(1,1) + 1);
              sign(C(2,1)-C(1,2)) * sqrt(C(3,3) - C(1,1) - C(2,2) + 1)];
end
```

## rotVecWithQuat

```matlab
function A_r = rotVecWithQuat(quat_AB,B_r)
  % Input: 
  % the orientation quaternion and the coordinate of the vector to be mapped
  % Output: the coordinates of the vector in the target frame
  C_AB = quatToRotMat(quat_AB);
  A_r = C_AB*B_r;
end
```


## pseudoInverseMat

```matlab
function [pinvA] = pseudoInverseMat(A, lambda)
    % Computes the Moore-Penrose pseudo-inverse of a matrix.
    % Input:
    %   A: Any m-by-n matrix.
    %   lambda: Regularization parameter.
    % Output:
    %   An n-by-m pseudo-inverse of the input.

    % Get the number of rows (m) and columns (n) of A
    [m, n] = size(A);

    % Compute the pseudo-inverse for both left and right cases
    if (m > n)
        % Compute the left pseudo-inverse.
        pinvA = (A' * A + lambda * lambda * eye(n, n)) \ A';
    elseif (m <= n)
        % Compute the right pseudo-inverse.
        pinvA = A' / (A * A' + lambda * lambda * eye(m, m));
    end
end

```

## rotMatToRotVec
    
```matlab
% Function to convert a rotation matrix to a rotational vector
function [phi] = rotMatToRotVec(C)
    % Input: a rotation matrix C
    % Output: the rotational vector which describes the rotation C

    th = acos(0.5 * (C(1, 1) + C(2, 2) + C(3, 3) - 1));
    if (abs(th) < eps)
        n = zeros(3, 1);
    else
        n = 1 / (2 * sin(th)) * [C(3, 2) - C(2, 3);
                                 C(1, 3) - C(3, 1);
                                 C(2, 1) - C(1, 2)];
    end
    phi = th * n;
end
```


## inverseKinematics

```matlab
function [ q ] = inverseKinematics(I_r_IE_des, C_IE_des, q_0, tol)
    % Input: desired end-effector position, desired end-effector orientation (rotation matrix),
    %        initial guess for joint angles, threshold for the stopping-criterion
    % Output: joint angles which match desired end-effector position and orientation

    % 0. Setup
    it = 0;
    max_it = 100;       % Set the maximum number of iterations. 
    lambda = 0.001;     % Damping factor.
    alpha = 0.5;        % Update rate

    close all;
    loadviz;

    % 1. start configuration
    q = q_0;

    % 2. Iterate until terminating condition.
    while (it==0 || (norm(dxe)>tol && it < max_it))
        % 3. evaluate Jacobian for current q
        I_J = [jointToPosJac_solution(q); ...
               jointToRotJac_solution(q)];

        % 4. Update the psuedo inverse
        I_J_pinv = pseudoInverseMat_solution(I_J, lambda);

        % 5. Find the end-effector configuration error vector
        % position error
        I_r_IE = jointToPosition_solution(q);
        dr = I_r_IE_des - I_r_IE; 
        % rotation error
        C_IE = jointToRotMat_solution(q);
        C_err = C_IE_des*C_IE';
        dph = rotMatToRotVec_solution(C_err); 
        % 6D error
        dxe = [dr; dph];

        % 6. Update the generalized coordinates
        q = q + alpha*I_J_pinv*dxe;

        % Update robot
        abbRobot.setJointPositions(q);
        drawnow;
        pause(0.1);

        it = it+1;
    end

    % Get final error (as for 5.)
    % position error
    I_r_IE = jointToPosition_solution(q);
    dr = I_r_IE_des - I_r_IE; 
    % rotation error
    C_IE = jointToRotMat_solution(q);
    C_err = C_IE_des*C_IE';
    dph = rotMatToRotVec_solution(C_err); 

    fprintf('Inverse kinematics terminated after %d iterations.\n', it);
    fprintf('Position error: %e.\n', norm(dr));
    fprintf('Attitude error: %e.\n', norm(dph));
end

```
