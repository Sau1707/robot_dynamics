function I_dJr_C = I_dJr_C_fun(in1,in2)
%I_DJR_C_FUN
%    I_DJR_C = I_DJR_C_FUN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    04-Nov-2020 09:25:52

dq1 = in2(1,:);
q1 = in1(1,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = dq1.*t2;
t5 = dq1.*t3;
t6 = -t5;
I_dJr_C = reshape([0.0,0.0,0.0,t6,t4,0.0,t6,t4,0.0],[3,3]);
