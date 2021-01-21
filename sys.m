function [ de ] = sys( t,e,K,A,B,xd_dd,yd_dd,zd_dd)

r=[xd_dd,yd_dd,zd_dd]'-K*e;
de=A*e+B*r;

end

