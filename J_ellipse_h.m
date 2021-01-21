function V = J_ellipse_h(q, xi, yi, zi, thi, hi, a, b, xc,yc,delta)

%%%%%%%%%%%%%%%%%%%%%%%%%%
% syms z h d t x y k
% ai=(z/2)*(tan(h+d)-tan(h-d));
% bi=z*tan(d)*(1+((tan(h+d)+tan(h-d))/2)^2)^0.5;
% xci=x+cos(k)*(z/2)*(tan(h+d)+tan(h-d));
% yci=y+sin(k)*(z/2)*(tan(h+d)+tan(h-d));
% R = [cos(t)  -sin(t);sin(t)  cos(t)];
% C=R*[ai*cos(k);bi*sin(k)]+[xci;yci];
% 
% p = matlabFunction(diff(C,h),'File','jh','Vars',[z h d t x y k]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find value of parameter t
q = q - [xc ; yc];
q = ROT(q, -thi);
tau = atan2( q(2)/b, q(1)/a );



V=jh(zi,hi,delta, thi,xi,yi,tau);
end