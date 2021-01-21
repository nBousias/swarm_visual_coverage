function B = B_m(x,y,z,dx,dy,dz,q,r,p,phi,psi,theta,u1,u2,u3,u4,Ix,Iy,Iz,g,m)
%B_M
%    B = B_M(X,Y,Z,DX,DY,DZ,Q,R,P,PHI,PSI,THETA,U1,U2,U3,U4,IX,IY,IZ,G,M)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    10-Feb-2020 16:52:50

t2 = cos(phi);
t3 = cos(psi);
t4 = sin(phi);
t5 = sin(psi);
t6 = sin(theta);
t7 = 1.0./m;
B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,-t7.*(t4.*t5+t2.*t3.*t6),-t7.*(t3.*t4-t2.*t5.*t6),-t2.*t7.*cos(theta),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./Ix,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./Iy,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./Iz],[12,4]);
