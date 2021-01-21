clf

X = [0];
Y = [0];
Z  = [4];
TH  = [0];
HI = [0];
delta=deg2rad(20);
zmin = 0.3;
zmax = 6;
uz=0.5;



PPC = 120;
% Radius for points on plots
disk_rad = 0.02;
% Vector for circle parameterization
t = linspace(0, 2*pi, PPC+1);
t = t(1:end-1); % remove duplicate last element
t = fliplr(t); % flip to create CW ordered circles
[a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,1);
axis_scale = [-0.5 3 -0.5 3];
sim = struct;
% ---------------- Region ----------------
% Bullo region
Xb=1.95*([ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ]-1.5);
Yb=1.96*([ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ]-0.92);
[Xb, Yb] = poly2cw(Xb, Yb);
region = [Xb ; Yb];
region_area = polyarea( Xb, Yb );
axis_scale = [-0.5 3 -0.5 3];
sim.region=region;
sim.axis = axis_scale;
sim.PPC = PPC;
sim.zmin = zmin;
sim.zmax = zmax;
sim.X = X;
sim.Y = Y;
sim.Z = Z;
sim.TH = TH;
sim.HI = HI;
sim.a = a;
sim.b = b;
sim.xc = xc;
sim.yc = yc;

fill3(uz*cos(t)+X*ones(1,length(t)),uz*sin(t)+Y*ones(1,length(t)),Z*ones(1,length(t)), [0.7 0.7 0.6]);



i=1;
 C{i} = ROT([(a(i)-0.2)*cos(t); (b(i)-0.2)*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
sim.C=C;
        hold on
        % Sensing patterns and cells

%             fill3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), [0.8 0.7 0.7]);
            plot3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), '--r');
        % Node positions and cones

            plot3( sim.X(i), sim.Y(i), sim.Z(i), 'ko' )
            plot3( [sim.X(i) sim.X(i)], [sim.Y(i) sim.Y(i)], [sim.Z(i) 0], 'k--.' )
            for j=1:70:sim.PPC
%                 plot3([sim.C{i}(1,j) sim.X(i)], [sim.C{i}(2,j) sim.Y(i)], [0 sim.Z(i)], 'r--');
            end

        plot3(0.5*cos(t),0.5*sin(t),ones(120,1)*(sim.Z(i)+uz),'k');
        plot3(0.5*cos(t),0.5*sin(t),ones(120,1)*(sim.Z(i)-uz),'k');
        for j=1:66:sim.PPC
                plot3([0.5*cos(t(j)) 0.5*cos(t(j))], [ 0.5*sin(t(j)) 0.5*sin(t(j))], [sim.Z(i)-uz sim.Z(i)+uz], 'k');
        end
        
        
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 sim.zmax])
        axis equal
        axis off
        
    
plot3(xc,yc,0,'k*');
text(X-0.2,Y-0.2,Z-0.2,'$X_i$', 'FontSize',14,'interpreter','latex');
text(-1.8,1.7,0,'$\Omega$', 'FontSize',28,'interpreter','latex');

text(-1.1,1.6,0,'$\Omega^s$', 'FontSize',28,'interpreter','latex');



%%%%%%%%%%%%%%%%222222222222222222



X = [0];
Y = [-1.29];
Z  = [4.5];
TH  = [0];
HI = [0];
delta=deg2rad(20);
uz=0.5;

[a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,1);
% ---------------- Region ----------------

sim.X = X;
sim.Y = Y;
sim.Z = Z;
sim.TH = TH;
sim.HI = HI;
sim.a = a;
sim.b = b;
sim.xc = xc;
sim.yc = yc;

i=1;
 C{i} = ROT([(a(i)-0.2)*cos(t); (b(i)-0.2)*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
sim.C=C;
        hold on
        % Sensing patterns and cells

%             fill3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), [0.8 0.7 0.7]);
plot3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), '--r');
        % Node positions and cones

            plot3( sim.X(i), sim.Y(i), sim.Z(i), 'ko' )
            plot3( [sim.X(i) sim.X(i)], [sim.Y(i) sim.Y(i)], [sim.Z(i) 0], 'k--.' )
            for j=1:70:sim.PPC
                %plot3([sim.C{i}(1,j) sim.X(i)], [sim.C{i}(2,j) sim.Y(i)], [0 sim.Z(i)], 'r--');
            end

        plot3(0.5*cos(t)+X*ones(1,length(t)),0.5*sin(t)+Y*ones(1,length(t)),ones(120,1)*(sim.Z(i)+uz),'k');
        plot3(0.5*cos(t)+X*ones(1,length(t)),0.5*sin(t)+Y*ones(1,length(t)),ones(120,1)*(sim.Z(i)-uz),'k');
        for j=1:66:sim.PPC
                plot3([0.5*cos(t(j))+X 0.5*cos(t(j))+X], [ 0.5*sin(t(j))+Y 0.5*sin(t(j))+Y], [sim.Z(i)-uz sim.Z(i)+uz], 'k');
        end
        
        
fill3(uz*cos(t)+X*ones(1,length(t)),uz*sin(t)+Y*ones(1,length(t)),Z*ones(1,length(t)), [0.7 0.7 0.9]);    
plot3(xc,yc,0,'k*');
text(X+0.2,Y+0.1,Z-0.2,'$X_i$', 'FontSize',14,'interpreter','latex');


text(X-0.25,Y-0.25,Z-0.2,'$\bf{r_i^q}$', 'FontSize',14,'interpreter','latex');
text(0.1,0.25,Z-0.2,'$\bf{r_i^q}$', 'FontSize',14,'interpreter','latex');

% plot3([0 0],[0 1],[4 4.5],'-r');

  
        
%         fill3( region(1,:),region(2,:), zeros(size(region)), [0.2 0.85 0.8]);
%         alpha(.5)
 % Plot region
        plot3_poly( [sim.region ; zeros(size(sim.region(1,:)))], 'k--' );
        plot3_AABB([sim.axis 0 sim.zmax], 'w.');
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(0, 90);
        axis([-3 3 -3 3 0 5.5])
        axis equal
        axis off
        hold on;

        
Xb2=1.5*([ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ]-1.51);
Yb2=1.5*([ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ]-0.86);
[Xb2, Yb2] = poly2cw(Xb2, Yb2);
region2 = [Xb2 ; Yb2];
sim.region=region;
plot3_poly( [sim.region ; zeros(size(sim.region(1,:)))], 'k--' );
fill3( region2(1,:),region2(2,:), zeros(size(region2)), [0.75 0.75 0.75]);
alpha(.6)


k=2;
for i=1:2:length(Xb2)-1
    dx=(Xb2(i)-Xb2(i+1))/k;
    dy=(Yb2(i)-Yb2(i+1))/k;
    for j=1:1
        x=Xb2(i)-(j-1)*dx;
        y=Yb2(i)-(j-1)*dy;
        plot_others(x,y,t,sim);
    end
end



