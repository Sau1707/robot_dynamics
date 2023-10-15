function [ q ] = inverseKinematics(I_r_IC_des, q_0, tol, params)
% Input: 
% I_r_IC_des: 3x1 desired position of the point C
% q_0: 3x1 initial guess for joint angles
% tol: 1x1 tolerance to use as termination criterion
%      The tolerance should be used as:
%      norm(I_r_IC_des - I_r_IC) < tol
% params: a struct of parameters
% Output:
% q: a vector of joint angles q (3x1) which achieves the desired
%    task-space position

% 0. Setup
it = 0;
max_it = params.max_it;       % Set the maximum number of iterations. 
lambda = params.lambda;       % Damping factor
alpha = params.alpha;         % Update rate

% 1. start configuration
q = q_0;

% implement your solution here ...

end