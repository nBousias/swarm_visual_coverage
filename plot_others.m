function []=plot_others(X,Y,t,sim)
Z  = [2.5];
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
alpha(.6)
end