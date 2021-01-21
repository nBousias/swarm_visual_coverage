clf

X = [0];
Y = [0];
Z  = [2.3];
TH  = [0];
HI = [0];
delta=deg2rad(20);
zmin = 0.3;
zmax = 2.3;
uz=0.2;


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
Xb=2.6*([ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ]-1);
Yb=2.6*([ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ]-1.9);
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

i=1;
 C{i} = ROT([(a(i)-0.2)*cos(t); (b(i)-0.2)*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
sim.C=C;
        hold on
        % Sensing patterns and cells

			plot3_poly( [sim.C{i} ; zeros(size(sim.C{i}(1,:)))], 'k--');
            fill3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), [0.8 0.7 0.7]);
            plot3((a)*cos(t),(b)*sin(t),zeros(120),'k');
        % Node positions and cones

            plot3( sim.X(i), sim.Y(i), sim.Z(i), 'ko' )
            plot3( [sim.X(i) sim.X(i)], [sim.Y(i) sim.Y(i)], [sim.Z(i) 0], 'k--.' )
            for j=1:70:sim.PPC
                plot3([sim.C{i}(1,j) sim.X(i)], [sim.C{i}(2,j) sim.Y(i)], [0 sim.Z(i)], 'r--');
            end

        
        
        plot3(0.2*cos(t),0.2*sin(t),ones(120,1)*(sim.Z(i)+uz),'k');
        plot3(0.2*cos(t),0.2*sin(t),ones(120,1)*(sim.Z(i)-uz),'k');
        for j=1:60:sim.PPC
                plot3([0.2*cos(t(j)) 0.2*cos(t(j))], [ 0.2*sin(t(j)) 0.2*sin(t(j))], [sim.Z(i)-uz sim.Z(i)+uz], 'k');
        end
        
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 sim.zmax])
        axis equal
        axis off
        
    
 
        TH  = [-pi/4];
        HI = [pi/4];
[a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,1);
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
r=0.2;
    C{i} = ROT([(a(i))*cos(t); (b(i))*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
    sim.C=C;
        hold on
        % Sensing patterns and cells

			plot3_poly( [sim.C{i} ; zeros(size(sim.C{i}(1,:)))], 'k--');
            
            
            C{i} = ROT([(a(i)-r)*cos(t); (b(i)-r)*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
        sim.C=C;
            fill3( sim.C{i}(1,:),sim.C{i}(2,:), zeros(size(sim.C{i}(1,:))), [0.8 0.7 0.7]);
            
plot3([X-1 xc],[Y-1 yc],[Z+1.7 0],'k--');
plot3([X X-1],[Y Y-1],zeros(2),'k');
plot3([X-0.8 X-1.5],[Y Y-0.5],zeros(2),'k');
plot3(xc,yc,0,'k*');


text(xc-0.23,yc-0.5,0,'$q_{i,c}$','interpreter','latex');
text(xc+0.6,yc+0.2,0,'$C_i^{gs}$','interpreter','latex');
text(X-1.5,Y-1.5,0,'$C_i^{gs}$','interpreter','latex');  
text(X-2,Y-0.5,0,'$C_i^{g}$','interpreter','latex');  
text(X-0.23,Y-0.1,0,'$q_{i,c}$','interpreter','latex'); 
text(X+0.4,Y+0.3,Z-0.2,'$X_i$','interpreter','latex');
text(-0.3,-3+0.3,0,'$x$','interpreter','latex');
text(3-0.3,0.3,0,'$y$','interpreter','latex');
text(-0.1,0.1,4,'$z$','interpreter','latex');
text(X-1,Y+0.3,Z-0.8,'$C_i^u$','interpreter','latex');
plot3([X-0.8 X-0.2],[Y+0.4 Y-0.3],[Z-0.7 Z],'k');

k=find(inpolygon(2*sin(t),2*cos(t),[0 0 xc+1],[0 -3 yc-1])==1);
plot3(2*sin(t(k)),2*cos(t(k)),zeros(length(k)));
text(0.4,-2.3,0,'$\theta_i$','interpreter','latex');
text(-0.5,-0.8,1.7,'$\delta_i$','interpreter','latex');
text(0.4,-2.3,1.5,'$h_i$','interpreter','latex');

text(4,0.5,0,'$\Omega$','interpreter','latex');

plot3(2*sin(t(57:60)), zeros(1,(4)),2*cos(t(57:60))+Z+1,'k');


        % Node positions and cones

            plot3( sim.X(i), sim.Y(i), sim.Z(i), 'ko' )
            plot3( [sim.X(i) sim.X(i)], [sim.Y(i) sim.Y(i)], [sim.Z(i) 0], 'k--.' )
            for j=1:35:sim.PPC
                plot3([sim.C{i}(1,j) sim.X(i)], [sim.C{i}(2,j) sim.Y(i)], [0 sim.Z(i)], 'r--');
            end

        plot3([0 0 0 0 3],[-3 0 0 0 0],[0 0 4 0 0],'k');
        plot3([X xc+1],[Y yc-1],[0 0],'k--');
        
        % Plot region
        
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 sim.zmax])
        ax.XAxisLocation = 'origin';
        ax.YAxisLocation = 'origin';
        axis equal

 % Plot region
        plot3_poly( [sim.region ; zeros(size(sim.region(1,:)))], 'k--' );
        plot3_AABB([sim.axis 0 sim.zmax], 'w.');
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 sim.zmax])
        axis equal
        axis off