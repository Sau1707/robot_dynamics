function I_Jr_C = I_Jr_C_fun(in1,in2)
%I_Jr_C_fun
%    I_Jr_C = I_Jr_C_fun(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    05-Nov-2023 13:21:58

q1 = in1(1,:);
t2 = cos(q1);
t3 = sin(q1);
I_Jr_C = reshape([0.0,0.0,1.0,t2,t3,0.0,t2,t3,0.0],[3,3]);
end
