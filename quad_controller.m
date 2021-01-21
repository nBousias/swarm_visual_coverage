function [u] = quad_controller(sim,x_quad,coef,t)
% u=K*x_quad;

Kd=sim.Kd;
Kp=sim.Kp;
Kr=sim.Kr;
Kw=sim.Kw;

m=sim.m;
g=sim.gr;
I=sim.J;


%states
x         = x_quad(1);
y         = x_quad(2);
z         = x_quad(3);
x_dot     = x_quad(4);
y_dot     = x_quad(5);
z_dot     = x_quad(6);
phi       = x_quad(7);
theta     = x_quad(8);
psi       = x_quad(9);
phi_dot   = x_quad(10);
theta_dot = x_quad(11);
psi_dot   = x_quad(12);

w=[phi_dot; theta_dot;psi_dot];

%desired trajectory
yaw_des = 0;
yaw_dot_des = 0;

r_T = coef(:,1)*t^5+ coef(:,2)*t^4+ coef(:,3)*t^3 + coef(:,4)*t^2+ coef(:,5)*t + coef(:,6);
r_dot_T = 5*coef(:,1)*t^4+ 4*coef(:,2)*t^3+ 3*coef(:,3)*t^2 + 2*coef(:,4)*t+ coef(:,5);
r_ddot_T = 20*coef(:,1)*t^3+ 12*coef(:,2)*t^2+ 6*coef(:,3)*t + 2*coef(:,4);

% r_dot_T=[0;0;0];
% r_T=[0;0;0];
% r_ddot_T = [0; 0; 0];



%rotation matrix
R=rotation_matrix(phi,theta,psi);

%velocity error
e_r_dot=[x_dot ; y_dot ; z_dot] -r_dot_T;
%position error
e_r= [x ; y ; z] -r_T;

r_ddot_des=r_ddot_T- Kd*e_r_dot-Kp*e_r;

F_des=m*r_ddot_des+[0;0;m*g];

u1=(R(:,3).')*F_des;

b3_des=F_des/norm(F_des);

a=[cos(yaw_des);sin(yaw_des);0];

b2_des=cross(b3_des,a)/norm(cross(b3_des,a));

R_des=[cross(b2_des,b3_des), b2_des, b3_des];

aux=0.5*( R_des'*R - R'*R_des );
e_R=[aux(3,2);aux(1,3);aux(2,1)];

e_W = w;

u2 = I*(- Kr*e_R - Kw*e_W);

% if u1<0
%     u1 = 0;
% end

u=[u1;u2];

aux2 = (sim.U2F*u)/sim.K_F;
rot_speeds = diag(sign(aux2))*sqrt(abs(aux2));
rot_speeds(rot_speeds > sim.rotor_speed_max) = sim.rotor_speed_max;
rot_speeds(rot_speeds < sim.rotor_speed_min) = sim.rotor_speed_min;


u = sim.F2U*(sim.K_F*rot_speeds.^2);


end





