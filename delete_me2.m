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
Xb=4*([ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ]-1);
Yb=4*([ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ]-1.5);
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

        % Node positions and cones
     
        
        plot3(0.2*cos(t),0.2*sin(t),ones(120,1)*(sim.Z(i)+uz),'k');
        plot3(0.2*cos(t),0.2*sin(t),ones(120,1)*(sim.Z(i)-uz),'k');
        for j=1:60:sim.PPC
                plot3([0.2*cos(t(j)) 0.2*cos(t(j))], [ 0.2*sin(t(j)) 0.2*sin(t(j))], [sim.Z(i)-uz sim.Z(i)+uz], 'k');
        end
        
    





text(xc-0.23,yc-0.5,0,'$q_{i,c}$','interpreter','latex');
text(xc-0.3,yc+0.7,0,'$C_i^{gs}$','interpreter','latex');

text(X-0.4,Y-0.4,0,'$q_{i,c}$','interpreter','latex'); 
text(X+0.4,Y+0.3,Z-0.2,'$X_i$','interpreter','latex');
text(X-0.4,Y+0.3,Z+0.3,'$C_i^u$','interpreter','latex');
text(-0.3,-3+0.3,0,'$x$','interpreter','latex');
text(3-0.3,0.3,0,'$y$','interpreter','latex');
text(-0.1,0.1,4,'$z$','interpreter','latex');



text(4,-0.7,0,'$\Omega$','interpreter','latex');




        % Node positions and cones

            plot3( sim.X(i), sim.Y(i), sim.Z(i), 'kh' )
            plot3( sim.X(i), sim.Y(i), sim.Z(i)+uz, 'bh' )
            plot3( sim.X(i), sim.Y(i), sim.Z(i)-uz, 'rh' )
            plot3( [sim.X(i) sim.X(i)], [sim.Y(i) sim.Y(i)], [sim.Z(i) 0], 'k--.' )
            for j=1:60:sim.PPC
                plot3([sim.C{i}(1,j) sim.X(i)], [sim.C{i}(2,j) sim.Y(i)], [0 sim.Z(i)+uz], 'b--');
            end
            
%             for j=1:60:sim.PPC
%                 plot3([C1{i}(1,j) sim.X(i)], [C1{i}(2,j) sim.Y(i)], [0 sim.Z(i)-uz], 'r--');
%             end
%             for j=1:60:sim.PPC
%                 plot3([C2{i}(1,j) sim.X(i)], [C2{i}(2,j) sim.Y(i)], [0 sim.Z(i)], 'k--');
%             end

        plot3([0 0 0 0 3],[-3 0 0 0 0],[0 0 4 0 0],'k');
        plot3([X xc+1],[Y yc-1],[0 0],'k--');
        
%         [xo,yo]=polybool('intersection',C{1}(1,:),C{1}(2,:),C1{1}(1,:),C1{1}(2,:));
%         fill3( xo,yo, zeros(size(xo)), [0.8 0.7 0.7]);
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
