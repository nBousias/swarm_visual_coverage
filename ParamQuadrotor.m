clear all
close all
m = 1;
Ixx = 8.1e-3;
Iyy = 8.1e-3;
Izz = 14.2e-3;
I = [Ixx 0 0;0 Iyy 0;0 0 Izz];
Jtp = 104e-6;
Ke = 6.3e-3;
Km = 6.3e-3;
L = 15e-6;
b = 54.2e-6;
l = 0.24;
g = 9.81;
n = 0.9;
N = 5.6;
h = 1e-3;
R = 0.6;
d = 1.1e-6; % drag factor
% Motor Controller Design
Kp_position = 5;
Ki_position = 0;
Kd_position = 10;
Kp_angle = 5*2;
Ki_angle = 0;
Kd_angle = 10*2;
Kp_motor = 1;
Ki_motor = 10;
Kd_motor = 0;
%w'=Apw+Bpv+Cp
Ap = -22.5;
Bp = 509;
Cp = 489;
a1 = Ap;
b1 = Bp;
c1 = 1;
d1 = 0;
sys_c_m = ss(a1,b1,c1,d1);
Gc_m = tf(sys_c_m);
sys_d_m = c2d(sys_c_m,h,'zoh');
Gd_m = tf(sys_d_m);
Cc_m = tf([Kp_motor Ki_motor],[1 0]);
sys_c_mctrl = ss(Cc_m);
sys_d_mctrl = c2d(sys_c_mctrl,h,'tustin');
Cd_m = tf(sys_d_mctrl);
LGd_m = Gd_m*Cd_m;
Td_m = feedback(LGd_m,1);
figure(1)
margin(LGd_m)
hold on
grid on
figure(2)
step(Td_m)
hold on
grid on
num = [1];
den = [1 0 0];
Gc_p = tf(num,den);
sys_c_p = ss(Gc_p);
sys_d_p = c2d(sys_c_p,h,'zoh');
Gd_p = tf(sys_d_p);
Cc_a = tf([Kp_angle Ki_angle],[1 0]);
sys_c_actrl = ss(Cc_a);
sys_d_actrl = c2d(sys_c_actrl,h,'tustin');
Cd_a = tf(sys_d_actrl);
