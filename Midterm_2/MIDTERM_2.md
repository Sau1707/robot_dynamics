## Basics

### Transpose

```matlab
A = [1 2 3; 4 5 6];
B = A'; % B = [1 4; 2 5; 3 6]
```

<br />

## Equation of motion:

$$ M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau + J_c(q)^TF_c $$

###  Mass matrix
$$M = \sum_{i=1}^{n_b} ({_AJ_{S_i}}^T \cdot m_i \cdot {_AJ_{S_i}} + {_BJ^T_{R_i}} \cdot {_B\Theta_{S_i}} \cdot {_BJ_{R_i}}) $$

```matlab
for k = 1:length(q)
    M = M + m{k} * I_J_pos{k}' * I_J_pos{k} ...
    + I_J_rot{k}' *R_Ik{k} *k_I_s{k} *R_Ik{k}' *I_J_rot{k};
end
```

### Gravity terms

$$g = \sum_{i=1}^{n_b} \left( -{_AJ_{S_i}}^T \cdot {_AF_{g,i}} \right)$$
```matlab
for k = 1:length(q)
    g = g − I_J_pos{k}' * m{k} * I_g_acc;
end
```

### Non linear terms

$b=$
$$\sum_{i=1}^{n_b} \left( {_AJ_{S_i}}^T \cdot m_A \cdot {_A\dot{J}_{S_i}} \cdot \dot{q} \\
+ {_BJ^T_{R_i}} \left( {_B\Theta_{S_i}} \cdot {_B\dot{J}_{R_i}} \cdot \dot{q} + {_B\Omega_{S_i}} \times {_B\Theta_{S_i}} \cdot {_B\Omega_{S_i}} \right) \right)$$

```matlab
for k=1:length(q)
    dJp = dAdt(I_J_pos{k}, q, dq);
    dJr = dAdt(I_J_rot{k}, q, dq);

    omega_i = I_J_rot{k} * dq;
    I_sk = simplify(R_Ik{k} * k_I_s{k} * R_Ik{k}');

    b = b + I_J_pos{k}' * m{k} * dJp * dq + ...
            I_J_rot{k}' * I_sk * dJr * dq + ...
            I_J_rot{k}' * cross(omega_i , I_sk * omega_i);
end
```

### Energy

```matlab
E_kin = 0.5 * dphi' * M * dphi;
E_pot = sym(0);
for k=1:length(phi)
    E_pot = E_pot − m{k} * I_g_acc' * [eye(3) zeros(3,1)] ...
    * T Ik{k} * [k r ks {k};1];
end

hamiltonian = E_kin + E_pot;
```

### $\ddot{q}$ 

$$ M \ddot{q} + b + g = \tau \qquad \Longrightarrow \qquad
\ddot{q} = M^{-1} \cdot (\tau − b − g)$$ 

```matlab
ddq = M \ (tau − b - g); % better
% or
ddq = inv(M) * (tau - g - b); % longer to compute
```

### $\tau^*$

$$ \tau^* = k_p (q^* − q) + k_d (\dot{q}^* − \dot{q})$$

```matlab
tau = kp * (q_des − q) ...
    + kd * (dq_des − dq)...
```

With gravity compensation:

$$ \tau^* = k_p (q^* − q) + k_d (\dot{q}^* − \dot{q}) + \hat{g}(q)$$

```matlab
tau = tau + g;
```


### joint−space dynamics to the task

```matlab
% Note: use pseudoInverseMat() function for lambda for stability 

JMinv = I_J_G / M;
lambda = pseudoInverseMat(JMinv * I_J_G');
mu = lambda * (JMinv * b − I_dJ_G * dq);
p = lambda * JMinv * g;
```