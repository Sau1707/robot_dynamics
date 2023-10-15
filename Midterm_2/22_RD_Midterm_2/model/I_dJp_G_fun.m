function I_dJp_G = I_dJp_G_fun(in1,in2)
%I_dJp_G_fun
%    I_dJp_G = I_dJp_G_fun(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    08-Nov-2022 01:35:33

dq1 = in2(2,:);
dq2 = in2(3,:);
q1 = in1(2,:);
q2 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = dq1+dq2;
t5 = q1+q2;
t8 = atan(7.150074692467946e-1);
t9 = 8.056283928194522e+16;
t6 = cos(t5);
t7 = sin(t5);
t10 = t5+t8;
t11 = t6.*1.950835906013399e-1;
t12 = t7.*1.950835906013399e-1;
t14 = t6.*2.728413324225067e-1;
t15 = t7.*2.728413324225067e-1;
t13 = -t12;
mt1 = [0.0,0.0,0.0,0.0,-dq2.*(t11+t15)-dq1.*(t2.*7.039550926810389e-2+t3.*4.44459753267812e-1+t11+t15),-dq2.*(t13+t14)-dq1.*(t2.*4.44459753267812e-1-t3.*7.039550926810389e-2+t13+t14),0.0];
mt2 = [t4.*t9.*sin(t10).*(-4.163336342344337e-18),t4.*t9.*cos(t10).*(-4.163336342344337e-18)];
I_dJp_G = reshape([mt1,mt2],3,3);
