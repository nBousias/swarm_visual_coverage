clear all;
close all;
clc;

%gravity accelaration
g = 9.780318;
m=0.75;

A=[  0     1     0     0     0     0
     0     0     0     0     0     0
     0     0     0     1     0     0
     0     0     0     0     0     0
     0     0     0     0     0     1
     0     0     0     0     0     0];
 
B=[  0     0     0
     1     0     0
     0     0     0
     0     1     0
     0     0     0
     0     0     1  ];



R=eye(3);
Q=eye(6);

[P,L,K]=care(A,B,Q,R); %RICCATI
K(1,2)=0.99;
K(2,4)=0.99;
K(3,6)=0.99;



T=0.01;
total_time=8;
time=0:T:total_time;
t=time;
len=length(t);

%--------DESIRED TRAJECTORY

xd=-2+2*cos(2*pi*t/20);
yd=2*sin(2*pi*t/20);
zd=3*(t/30).^2-2*(t/30).^3;

xd_d=-2*2*pi/20*sin(2*pi*t/20);
yd_d=2*2*pi/20*cos(2*pi*t/20);
zd_d=(6/30)*(t/30)-(6/30)*(t/30).^2;

xd_dd=-2*2*pi/20*2*pi/20*cos(2*pi*t/20);
yd_dd=-2*2*pi/20*2*pi/20*sin(2*pi*t/20);
zd_dd=(6/30)*(1/30)-(12/30)*(1/30)*(t/30);

% xd=2*sin(2*pi*t/20);
% yd=5*(t/40).^2-2*(t/40).^3;
% zd=5*(t/40).^2-2*(t/40).^3;
% 
% xd_d=2*2*pi/20*cos(2*pi*t/20);
% yd_d=(10/40)*(t/40)-(6/40)*(t/40).^2;
% zd_d=(10/40)*(t/40)-(6/40)*(t/40).^2;
% 
% xd_dd=-2*2*pi/20*2*pi/20*sin(2*pi*t/20);
% yd_dd=(10/40)*(1/40)-(12/40)*(1/40)*(t/40);
% zd_dd=(10/40)*(1/40)-(12/40)*(1/40)*(t/40);

x=[0];
y=[0];
z=[0];

t_initial=0;
error=[];
t=[];
r=[];

de_in=[0 0 0 0 0 0];

xq=[];
yq=[];
zq=[];

for i=1:len
    
    e_initial=[xd(i)-x(end),de_in(1),yd(i)-y(end),de_in(3),zd(i)-z(end),de_in(5)]';
    
    [tt,er]=ode45(@(t,e) sys(t,e,K,A,B,xd_dd(i),yd_dd(i),zd_dd(i)),[t_initial;t_initial+T],e_initial);
    
    t=[t;tt];
    error=[error; er];
    t_initial=t_initial+T;
    
    x=[x; xd(i)*ones(length(er),1)-er(:,1)];
    y=[y; yd(i)*ones(length(er),1)-er(:,3)];
    z=[z; zd(i)*ones(length(er),1)-er(:,5)];
    
    xq=[xq; xd(i)-er(end,1)];
    yq=[yq; yd(i)-er(end,3)];
    zq=[zq; zd(i)-er(end:5)];

%   r=[r [xd_dd(i)*ones(length(er),1),yd_dd(i)*ones(length(er),1),zd_dd(i)*ones(length(er),1)]'-K*er'];
    r=[r [xd_dd(i),yd_dd(i),zd_dd(i)]'-K*er(end,:)'];
    
    de_in=(A-B*K)*er(length(er),:)'+B*[xd_dd(i),yd_dd(i),zd_dd(i)]';    
end


% %plot positioning errors
% figure;
% plot(t,[error(:,1) error(:,3) error(:,5)]);
% % axis([0 max(t) 0 max(error)]);
% title('error');
% grid on;
% legend('x_{error}','y_{error}','z_{error}');

%plot trajectories
f2=figure;

subplot(2,1,1);
plot3([xd(1) xd(end)],[y(1) yd(end)],[zd(1) zd(end)],'*r');
grid on;
hold on;
plot3(xd,yd,zd,'--r');
hold on;
plot3([x(1) x(end)],[y(1) y(end)],[z(1) z(end)],'ok');
hold on;
plot3(x,y,z,'k');


%%%%%%%%%%%%%%%%%%
rx=r(1,:);
ry=r(2,:);
rz=r(3,:);

phid=atan2(-ry,(rz+g));
thetad=atan2(rx.*cos(phid),(rz+g));
Fd=m*(rz+g)./(cos(thetad).*cos(phid));

psid=[0];
k=0;
for i = 1:len-1
%     dx_dt = (xq(i+1)-xq(i))/(t(i+1)-t(i)) ;
%     dy_dt = (yq(i+1)-yq(i))/(t(i+1)-t(i)) ;
%     p=atan2d(dy_dt,dx_dt);
    p=atan2(yd_d(i),xd_d(i));
    if (abs(p-psid(end))>1.75*pi)
        k=k+1;
    end
    psid=[psid deg2rad(p+k*2*pi)];
end


phid_d=[0];
psid_d=[0];
thetad_d=[0];
phid_dd=[0];
psid_dd=[0];
thetad_dd=[0];
for i=1:len-1
    %desired roll angular velocity
    phid_d=[phid_d (phid(i+1)-phid(i))/T];
    %desired roll angular acceleration
    phid_dd=[phid_dd (phid_d(i+1)-phid_d(i))/T];
    
    %desired pitch angular velocity
    thetad_d=[thetad_d (thetad(i+1)-thetad(i))/T];
    %desired pitch angular acceleration
    thetad_dd=[thetad_dd (thetad_d(i+1)-thetad_d(i))/T];
    
    %desired yaw angular velocity
    psid_d=[psid_d (psid(i+1)-psid(i))/T];
    %desired yaw angular acceleration
    psid_dd=[psid_dd (psid_d(i+1)-psid_d(i))/T];
    
    end
    


ro=0.1;
phi=[0];
psi=[0];
theta=[0];

t_initial=0;
error=[];
t=[];
r=[];

de_in=[0 0 0 0 0 0];

K1=K(1,1);K2=K(1,2);tss=10/K2;
% K2=10/(ro*min(tss)); K1=power(K2/2,0.5);
K_r=[K1/10 2*K2 0 0 0 0; 0 0 K1/10 2*K2 0 0; 0 0 0 0 K1/3 K2*5];

% [P,L,K_r]=care(A,B,0.1*Q,10*R); %RICCATI

for i=1:len
    
    e_initial=[phid(i)-phi(end),de_in(1),thetad(i)-theta(end),de_in(3),psid(i)-psi(end),de_in(5)]';
    
    [tt,er]=ode45(@(t,e) sys(t,e,K_r,A,B,phid_dd(i),thetad_dd(i),psid_dd(i)),[t_initial;t_initial+T],e_initial);
    
    t=[t;tt(end)];
    error=[error; er(end,:)];
    t_initial=t_initial+T;
    
    phi=[phi; phid(i)-er(end,1)];
    theta=[theta; thetad(i)-er(end,3)];
    psi=[psi; psid(i)-er(end,5)];
        
    de_in=(A-B*K_r)*er(length(er),:)'+B*[phid_dd(i),thetad_dd(i),psid_dd(i)]';    
end


% %plot positioning errors
% figure;
% plot(t,[rad2deg(((error(:,1)))) rad2deg(((error(:,3)))) rad2deg(error(:,5))]);
% % axis([0 max(t) 0 max(error)]);
% title('error');
% grid on;
% legend('\phi_{error}','\theta_{error}','\psi_{error}');

% %plot trajectories
% figure;
% 
% title('Orientation tracking');
% 
% subplot(3,1,1);
% plot(t, rad2deg(phid),'--r');
% hold on;
% plot(t, rad2deg(phi(1:end-1)),'k');
% ylabel('\phi');
% legend('Desired angles','Quadcopter anlges');
% subplot(3,1,2);
% plot(t, rad2deg(thetad),'--r');
% hold on;
% plot(t, rad2deg(theta(1:end-1)),'k');
% ylabel('\theta');
% subplot(3,1,3);
% plot(t, rad2deg(psid),'--r');
% hold on;
% plot(t, rad2deg(psi(1:end-1)),'k');
% ylabel('\psi');
% xlabel('t');

quadrotor_plot(f2,[x(1) y(1) z(1)],[phi(1) theta(1) psi(1)]);
quadrotor_plot(f2,[x(end) y(end) z(end)],[phid(end) thetad(end) psid(end)]);
% legend('Actual','Nominal');
% title('Trajectory tracking');
xlabel('x');ylabel('y');zlabel('z');
view(30,40);














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R=0.0001*eye(3);Q=5*eye(6);

[P,L,K]=care(A,B,Q,R); %RICCATI
K(1,2)=0.99;
K(2,4)=0.99;
K(3,6)=0.99;



T=0.01;
total_time=8;
time=0:T:total_time;
t=time;
len=length(t);

%--------DESIRED TRAJECTORY

xd=-2+2*cos(2*pi*t/20);
yd=2*sin(2*pi*t/20);
zd=3*(t/30).^2-2*(t/30).^3;

xd_d=-2*2*pi/20*sin(2*pi*t/20);
yd_d=2*2*pi/20*cos(2*pi*t/20);
zd_d=(6/30)*(t/30)-(6/30)*(t/30).^2;

xd_dd=-2*2*pi/20*2*pi/20*cos(2*pi*t/20);
yd_dd=-2*2*pi/20*2*pi/20*sin(2*pi*t/20);
zd_dd=(6/30)*(1/30)-(12/30)*(1/30)*(t/30);

% xd=2*sin(2*pi*t/20);
% yd=5*(t/40).^2-2*(t/40).^3;
% zd=5*(t/40).^2-2*(t/40).^3;
% 
% xd_d=2*2*pi/20*cos(2*pi*t/20);
% yd_d=(10/40)*(t/40)-(6/40)*(t/40).^2;
% zd_d=(10/40)*(t/40)-(6/40)*(t/40).^2;
% 
% xd_dd=-2*2*pi/20*2*pi/20*sin(2*pi*t/20);
% yd_dd=(10/40)*(1/40)-(12/40)*(1/40)*(t/40);
% zd_dd=(10/40)*(1/40)-(12/40)*(1/40)*(t/40);

x=[0];
y=[0];
z=[0];

t_initial=0;
error=[];
t=[];
r=[];

de_in=[0 0 0 0 0 0];

xq=[];
yq=[];
zq=[];

for i=1:len
    
    e_initial=[xd(i)-x(end),de_in(1),yd(i)-y(end),de_in(3),zd(i)-z(end),de_in(5)]';
    
    [tt,er]=ode45(@(t,e) sys(t,e,K,A,B,xd_dd(i),yd_dd(i),zd_dd(i)),[t_initial;t_initial+T],e_initial);
    
    t=[t;tt];
    error=[error; er];
    t_initial=t_initial+T;
    
    x=[x; xd(i)*ones(length(er),1)-er(:,1)];
    y=[y; yd(i)*ones(length(er),1)-er(:,3)];
    z=[z; zd(i)*ones(length(er),1)-er(:,5)];
    
    xq=[xq; xd(i)-er(end,1)];
    yq=[yq; yd(i)-er(end,3)];
    zq=[zq; zd(i)-er(end:5)];

%   r=[r [xd_dd(i)*ones(length(er),1),yd_dd(i)*ones(length(er),1),zd_dd(i)*ones(length(er),1)]'-K*er'];
    r=[r [xd_dd(i),yd_dd(i),zd_dd(i)]'-K*er(end,:)'];
    
    de_in=(A-B*K)*er(length(er),:)'+B*[xd_dd(i),yd_dd(i),zd_dd(i)]';    
end


subplot(2,1,2);
plot3([xd(1) xd(end)],[y(1) yd(end)],[zd(1) zd(end)],'*r');
grid on;
hold on;
plot3(xd,yd,zd,'--r');
hold on;
plot3([x(1) x(end)],[y(1) y(end)],[z(1) z(end)],'ok');
hold on;
plot3(x,y,z,'k');

%%%%%%%%%%%%%%%%%%
rx=r(1,:);
ry=r(2,:);
rz=r(3,:);

phid=atan2(-ry,(rz+g));
thetad=atan2(rx.*cos(phid),(rz+g));
Fd=m*(rz+g)./(cos(thetad).*cos(phid));

psid=[0];
k=0;
for i = 1:len-1
%     dx_dt = (xq(i+1)-xq(i))/(t(i+1)-t(i)) ;
%     dy_dt = (yq(i+1)-yq(i))/(t(i+1)-t(i)) ;
%     p=atan2d(dy_dt,dx_dt);
    p=atan2(yd_d(i),xd_d(i));
    if (abs(p-psid(end))>1.75*pi)
        k=k+1;
    end
    psid=[psid deg2rad(p+k*2*pi)];
end


phid_d=[0];
psid_d=[0];
thetad_d=[0];
phid_dd=[0];
psid_dd=[0];
thetad_dd=[0];
for i=1:len-1
    %desired roll angular velocity
    phid_d=[phid_d (phid(i+1)-phid(i))/T];
    %desired roll angular acceleration
    phid_dd=[phid_dd (phid_d(i+1)-phid_d(i))/T];
    
    %desired pitch angular velocity
    thetad_d=[thetad_d (thetad(i+1)-thetad(i))/T];
    %desired pitch angular acceleration
    thetad_dd=[thetad_dd (thetad_d(i+1)-thetad_d(i))/T];
    
    %desired yaw angular velocity
    psid_d=[psid_d (psid(i+1)-psid(i))/T];
    %desired yaw angular acceleration
    psid_dd=[psid_dd (psid_d(i+1)-psid_d(i))/T];
    
    end
    


ro=0.1;
phi=[0];
psi=[0];
theta=[0];

t_initial=0;
error=[];
t=[];
r=[];

de_in=[0 0 0 0 0 0];

K1=K(1,1);K2=K(1,2);tss=10/K2;
% K2=10/(ro*min(tss)); K1=power(K2/2,0.5);
K_r=[K1/10 2*K2 0 0 0 0; 0 0 K1/10 2*K2 0 0; 0 0 0 0 K1/3 K2*5];

% [P,L,K_r]=care(A,B,0.1*Q,10*R); %RICCATI

for i=1:len
    
    e_initial=[phid(i)-phi(end),de_in(1),thetad(i)-theta(end),de_in(3),psid(i)-psi(end),de_in(5)]';
    
    [tt,er]=ode45(@(t,e) sys(t,e,K_r,A,B,phid_dd(i),thetad_dd(i),psid_dd(i)),[t_initial;t_initial+T],e_initial);
    
    t=[t;tt(end)];
    error=[error; er(end,:)];
    t_initial=t_initial+T;
    
    phi=[phi; phid(i)-er(end,1)];
    theta=[theta; thetad(i)-er(end,3)];
    psi=[psi; psid(i)-er(end,5)];
        
    de_in=(A-B*K_r)*er(length(er),:)'+B*[phid_dd(i),thetad_dd(i),psid_dd(i)]';    
end


% %plot positioning errors
% figure;
% plot(t,[rad2deg(((error(:,1)))) rad2deg(((error(:,3)))) rad2deg(error(:,5))]);
% % axis([0 max(t) 0 max(error)]);
% title('error');
% grid on;
% legend('\phi_{error}','\theta_{error}','\psi_{error}');

% %plot trajectories
% figure;
% 
% title('Orientation tracking');
% 
% subplot(3,1,1);
% plot(t, rad2deg(phid),'--r');
% hold on;
% plot(t, rad2deg(phi(1:end-1)),'k');
% ylabel('\phi');
% legend('Desired angles','Quadcopter anlges');
% subplot(3,1,2);
% plot(t, rad2deg(thetad),'--r');
% hold on;
% plot(t, rad2deg(theta(1:end-1)),'k');
% ylabel('\theta');
% subplot(3,1,3);
% plot(t, rad2deg(psid),'--r');
% hold on;
% plot(t, rad2deg(psi(1:end-1)),'k');
% ylabel('\psi');
% xlabel('t');

quadrotor_plot(f2,[x(1) y(1) z(1)],[phi(1) theta(1) psi(1)]);
quadrotor_plot(f2,[x(end) y(end) z(end)],[phid(end) thetad(end) psid(end)]);
% legend('Actual','Nominal');
% title('Trajectory tracking');
xlabel('x');ylabel('y');zlabel('z');
view(30,40);