function [sense] = min_distance_point(xi,yi,zi,thi,h,delta,t,r)

a=abs((zi/2)*(tan(h+delta)-tan(h-delta)));
    
b=abs(zi*tan(delta)*(1+((tan(h+delta)+tan(h-delta))/2)^2)^0.5);

xc=xi+cos(thi)*(zi/2)*(tan(h+delta)+tan(h-delta));

yc=yi+sin(thi)*(zi/2)*(tan(h+delta)+tan(h-delta));

sense=ROT([(a-r)*cos(t); (b-r)*sin(t)],thi)+ ([xc;yc]*ones(1,length(t)));

%q=sense(find(min((sense(1,:)-xi*ones(1,length(t))).^2+(sense(2,:)-yi*ones(1,length(t))).^2)),:);

end

