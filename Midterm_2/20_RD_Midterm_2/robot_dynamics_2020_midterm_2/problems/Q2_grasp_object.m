% generate equations of motion
function eom = Q2_grasp_object(gc, kin, params, jac)
% By calling:
%   eom = generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion.
%
% Inputs:
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : Struct containing symbolic expresses for the kinematics
%   - params    : Struct with parameters
%   - jac       : Struct containing symbolic expresses for the jacobians
%   
% Output:
%   - eom       : Struct with fields {M, b, g}, implementing the system
%   dynamics with the grasped object
%

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (3x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (3x1 cell)->(3x3 sym)

k_I_s = params.k_I_s;      % Inertia tensor of body k in frame k (3x1 cell)->(3x3 sym)
m = params.m;              % Mass of body k (3x1 cell)->(1x1 double)
I_g_acc = params.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = params.k_r_ks;    % CoM location of body k in frame k (3x1 cell)->(3x1 double)

I_Jp = jac.I_Jp;    % CoM Positional Jacobian in frame I (3x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr; % CoM Rotational Jacobian in frame I (3x1 cell)->(3x6 sym)

I_Jp_E = jac.I_Jp_E; % Positional Jacobian of end effector
I_Jr_E = jac.I_Jr_E; % Rotational Jacobian of end effector
T_IE = kin.T_IE;     % Homogeneous transforms of end effector
R_IE = kin.R_IE;     % Rotation matrix of end effector

object_m = params.m_o;  % object's mass
object_I = params.E_I_o;  % object's rotational inertia tensor in end-effector frame

% You can use the dynamics without object as a starting point.
M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

%% Compute mass matrix
fprintf('Adapt mass matrix M... ');
%TODO: Implement M = ...;
M = M + ...
        I_Jp{3}'*params.m_o*I_Jp{3} + ...
        I_Jr{3}'*R_Ik{3}*params.E_I_o*R_Ik{3}'*I_Jr{3};
fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
%TODO: Implement g = ...;
g = g - ...
        I_Jp{3}'*params.m_o*I_g_acc;
fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
%TODO: Implement b = ...;
    I_dJp = dAdt(I_Jp{3},q,dq);
    I_dJr = dAdt(I_Jr{3},q,dq);
    I_omega = I_Jr{3}*dq;
    b = b + ...
        I_Jp{3}'*params.m_o*I_dJp*dq + ...
        I_Jr{3}'*R_Ik{3}*params.E_I_o*R_Ik{3}'*I_dJr*dq + ...
        I_Jr{3}'*cross(I_omega,(R_Ik{3}*params.E_I_o*R_Ik{3}'*I_omega));
fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end
