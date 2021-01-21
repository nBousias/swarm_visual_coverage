function A = A_m(x,y,z,dx,dy,dz,q,r,p,phi,psi,theta,u1,u2,u3,u4,Ix,Iy,Iz,g,m)
%A_M
%    A = A_M(X,Y,Z,DX,DY,DZ,Q,R,P,PHI,PSI,THETA,U1,U2,U3,U4,IX,IY,IZ,G,M)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    10-Feb-2020 16:52:50

t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t10 = -Iy;
t11 = -Iz;
t12 = 1.0./Ix;
t13 = 1.0./Iy;
t14 = 1.0./Iz;
t15 = 1.0./m;
t9 = t8.^2;
t16 = 1.0./t4;
t18 = Ix+t10;
t19 = Ix+t11;
t20 = Iy+t11;
t17 = t16.^2;
t21 = t9+1.0;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t15.*u1.*(t3.*t5-t2.*t6.*t7),t15.*u1.*(t5.*t6+t2.*t3.*t7),0.0,0.0,0.0,0.0,0.0,0.0,0.0,q.*t5.*t7.*t17+r.*t2.*t7.*t17,0.0,q.*t5.*t21+r.*t2.*t21,-t2.*t3.*t4.*t15.*u1,t2.*t4.*t6.*t15.*u1,t2.*t7.*t15.*u1,0.0,0.0,0.0,0.0,0.0,0.0,q.*t2.*t16-r.*t5.*t16,-q.*t5-r.*t2,q.*t2.*t8-r.*t5.*t8,-t15.*u1.*(t2.*t6-t3.*t5.*t7),-t15.*u1.*(t2.*t3+t5.*t6.*t7),t4.*t5.*t15.*u1,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,-r.*t13.*t19,q.*t14.*t18,0.0,0.0,0.0,t5.*t16,t2,t5.*t8,0.0,0.0,0.0,r.*t12.*t20,0.0,p.*t14.*t18,0.0,0.0,0.0,t2.*t16,-t5,t2.*t8,0.0,0.0,0.0,q.*t12.*t20,-p.*t13.*t19,0.0],[12,12]);
