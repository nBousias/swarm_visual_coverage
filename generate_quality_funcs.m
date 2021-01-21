close all
clear variables

syms x_i z_i z_min z_max h_i delta_i delta_min delta_max
assume([z_i z_min z_max h_i delta_i delta_min delta_max],'real')

% Function for the distance from the MAA to the sensing ellipse center
d = @(z,h,delta) z .* sqrt(1 + (tan(h + delta) + tan(h - delta)).^2 ./ 4);

% Function for old quality polynomial
f_old = @(x_i,x_min,x_max) ((x_i - x_min).^2 - (x_max - x_min).^2).^2 ./ (x_max - x_min).^4;

h_max = pi/2 - delta_max;
d_i = d(z_i, h_i, delta_i);
d_min = z_min;
d_max = z_max;
% Original, no delta
%f = 1 - tanh((d_i - d_min) ./ (d_max - d_min));
% Alternate
%f = (delta_max - delta_i) ./ (delta_max - delta_min) .* (1 - tanh((d_i - d_min) ./ (d_max - d_min)));
% % Alternate 2
% f =f_old(z_i,z_min,z_max);
% Alternate 2
f = 1/3 .* f_old(z_i,z_min,z_max) + 1/3 .* f_old(delta_i,delta_min,delta_max) + 1/3 .* f_old(h_i,0,h_max);

% Derivatives
dfz = diff(f,z_i);
dfh = diff(f,h_i);
dfdelta = diff(f,delta_i);

% Convert expressions to Matlab functions
matlabFunction(f_old,'File','f_old','Vars',[x_i x_min x_max]);
matlabFunction(f,'File','fu','Vars',[z_i h_i delta_i z_min z_max delta_min delta_max]);
matlabFunction(dfz,'File','dfuz','Vars',[z_i h_i delta_i z_min z_max delta_min delta_max]);
matlabFunction(dfh,'File','dfuh','Vars',[z_i h_i delta_i z_min z_max delta_min delta_max]);
matlabFunction(dfdelta,'File','dfud','Vars',[z_i h_i delta_i z_min z_max delta_min delta_max]);

% Plot functions
zmin = 0.3;
zmax = 2.3;
deltamin = deg2rad(1);
deltamax = deg2rad(45);
hmax = deg2rad(45);

figure
subplot(1,4,1)
z = linspace(zmin, zmax);
y = fu(z, 0, deltamin, zmin, zmax, deltamin, deltamax);
plot(z,y);
xlabel('z_i')
ylabel('f')

subplot(1,4,2)
del = linspace(deltamin, deltamax);
y = fu(zmin, 0, del, zmin, zmax, deltamin, deltamax);
plot(del,y);
xlabel('\delta_i')

subplot(1,4,3)
tilt = linspace(0, hmax);
y = fu(zmin, tilt, deltamin, zmin, zmax, deltamin, deltamax);
plot(tilt,y);
xlabel('h_i')

subplot(1,4,4)
y = fu(z, tilt, del, zmin, zmax, deltamin, deltamax);
plot(y);
xlabel('all')


% y1=f_old(z,zmin,zmax);
% y2=f_old(del,deltamin,deltamax);
% y3=f_old(tilt,0,hmax);
% plot(z,y1,'k');
% hold on;
% plot(del,y2,'r');
% hold on;
% plot(tilt,y3,'Color',[0.2 0.6 0.3]);
% h=legend('$p\left(z_i \; ; \ z^{\min}, \ z^{\max}\right)$','$p\left(\delta_i \; ; \ \delta^{\min}, \ \delta^{\max}\right)$','$p\left(h_i \; ; \ 0, \ h^{\max}\right)$'); 
% set(h,'Interpreter','latex')
% ylabel('p'); h=xlabel('$z_i,h_i,\delta_i$');
% set(h,'Interpreter','latex')






