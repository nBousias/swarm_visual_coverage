close all;
clear all;
clc;

%%
syms x y z dx dy dz q r p phi psi theta u1 u2 u3 u4 Ix Iy Iz g m

f=[dx, dy, dz, q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta)), q*cos(phi)-r*sin(phi), ...
    p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta), -1/m*(sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta))*u1, ...
    -1/m*(cos(psi)*sin(phi)-cos(phi)*sin(psi)*sin(theta))*u1, g-1/m*(cos(phi)*cos(theta))*u1, ...
    ((Iy-Iz)/Ix)*q*r+u2/Ix, ((Iz-Ix)/Iy)*p*r+u3/Iy, ((Ix-Iy)/Iz)*q*p+u4/Iz];


A=jacobian(f, [x y z psi theta phi dx dy dz p q r])
B=jacobian(f, [u1 u2 u3 u4])
% 
% A0=subs(A)
% B0=subs(B)

% Convert expressions to Matlab functions
matlabFunction(A,'File','A_m','Vars',[x y z dx dy dz q r p phi psi theta u1 u2 u3 u4 Ix Iy Iz g m]);
matlabFunction(B,'File','B_m','Vars',[x y z dx dy dz q r p phi psi theta u1 u2 u3 u4 Ix Iy Iz g m]);


%%
 
%define model parameters
m=0.75;
g=9.81;

Iy = 8.1e-3;
Iz = 14.2e-3;
Ix = 8.1e-3;
   
phi=deg2rad(0);
theta=deg2rad(0);
psi=deg2rad(0);
p=0;
q=0;
r=0;
dx=0;
dy=0;
dz=0;
x=0;
y=0;
z=0;

u1=m*g;
u2=0;
u3=0;
u4=0;


A0=A_m(x,y, z, dx, dy, dz, q, r, p, phi, psi, theta, u1, u2, u3, u4, Ix, Iy, Iz, g, m)
B0=B_m(x,y, z, dx, dy, dz, q, r, p, phi, psi, theta, u1, u2, u3, u4, Ix, Iy, Iz, g, m)

i=1;
for phi=[deg2rad(10),deg2rad(-10)]
   for theta=[deg2rad(10),deg2rad(-10)]
       fprintf('Roll angle:%.1f, Pitch angle: %.1f \n\n',[phi,theta])
       DA{i}=A_m(x,y, z, dx, dy, dz, q, r, p, phi, psi, theta, u1, u2, u3, u4, Ix, Iy, Iz, g, m)-A0;
       DB{i}=B_m(x,y, z, dx, dy, dz, q, r, p, phi, psi, theta, u1, u2, u3, u4, Ix, Iy, Iz, g, m)-B0;
       i=i+1;
   end
end
    
save('system_matrices.mat','A0','B0','DA','DB')
    
    
    
    
    
