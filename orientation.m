function [ x,y,z,error_angles] = orientation( s,Tstep,A,B,K,xd,yd,zd,x,y,z,uxd,uyd,uzd,x_d,y_d,z_d,axd,ayd,azd,x_dd,y_dd,z_dd)

        error=[xd-x;uxd-x_d;yd-y;uyd-y_d;zd-z;uzd-z_d];

        [~,e]=ode45(@(tt,er) sys(tt,er,K,A,B,axd,ayd,azd),[(s-1)*Tstep;s*Tstep],error);
        
        x=xd-e(1);
        y=yd-e(3);
        z=zd-e(5);    
        error_angles=[e(1),e(3),e(5)];
end

