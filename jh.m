function out1 = jh(z,h,d,t,x,y,k)
%JH
%    OUT1 = JH(Z,H,D,T,X,Y,K)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    06-Aug-2018 02:05:31

t5 = d-h;
t2 = tan(t5);
t7 = d+h;
t3 = tan(t7);
t4 = cos(k);
t6 = t2.^2;
t8 = t3.^2;
t10 = t3.*(1.0./2.0);
t11 = t2.*(1.0./2.0);
t9 = -t10+t11;
t12 = sin(k);
t13 = t6+t8+2.0;
t14 = sin(t);
t15 = t6-t8;
t16 = cos(t);
t17 = tan(d);
t18 = t10-t11;
t19 = t6.*(1.0./2.0);
t20 = t8.*(1.0./2.0);
t21 = t19+t20+1.0;
out1 = [t4.*t13.*z.*(1.0./2.0)-t4.*t15.*t16.*z.*(1.0./2.0)-t12.*t14.*t17.*t18.*t21.*z.*1.0./sqrt(t9.^2+1.0);t12.*t13.*z.*(1.0./2.0)-t4.*t14.*t15.*z.*(1.0./2.0)+t12.*t16.*t17.*t18.*t21.*z.*1.0./sqrt(t18.^2+1.0)];
