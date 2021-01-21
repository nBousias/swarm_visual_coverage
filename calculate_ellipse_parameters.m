function [ai,bi,xci,yci] = calculate_ellipse_parameters( h,theta,z,delta,xi,yi,N )

ai=[];
bi=[];
xci=[];
yci=[];

for i=1:N

    ai=[ai abs((z(i)/2)*(tan(h(i)+delta(i))-tan(h(i)-delta(i))))];
    
    bi=[bi abs(z(i)*tan(delta(i))*(1+((tan(h(i)+delta(i))+tan(h(i)-delta(i)))/2)^2)^0.5)];

    xci=[xci xi(i)+cos(theta(i))*(z(i)/2)*(tan(h(i)+delta(i))+tan(h(i)-delta(i)))];

    yci=[yci yi(i)+sin(theta(i))*(z(i)/2)*(tan(h(i)+delta(i))+tan(h(i)-delta(i)))];

end


end

