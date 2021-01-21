function plot_MAA(fig,sim )

if sim.PLOT_STATE_3D || sim.PLOT_STATE_2D || sim.PLOT_STATE_QUALITY || sim.PLOT_STATE_PHI
    
    
    
    % ----------------- Plot network 2D -----------------
    if sim.PLOT_STATE_2D
        clf
        hold on
        % Region
        plot_poly( sim.region, 'k');
        % Sensing patterns and cells
		for i=1:sim.N
			plot_poly( sim.C{i}, 'r--');
			plot_poly( sim.W{i}, 'k');
		end
		
        % Node positions
        for i=1:sim.N
%                 tmpc = [sim.X(i) + disk_rad * cos(t) ; sim.Y(i) + disk_rad * sin(t)];
%                 fill( tmpc(1,:), tmpc(2,:), 'k', 'EdgeColor', 'none' );
            plot( sim.X(i), sim.Y(i), 'k.' )
            hold on
        end
        
        plot_AABB(sim.axis, 'w.');

        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        axis(sim.axis)
        axis equal
        axis off

        if sim.SAVE_PLOTS
            fname = strcat( '~/Frames/', sprintf('2D_frame_%d.png', sim.s) );
            print(fname,'-dpng','-r600');
%             saveas(gcf, fname);
        else
            pause(0.01);
        end
    end

    
    
    % ----------------- Plot network 3D -----------------
    if sim.PLOT_STATE_3D
        clf
        subplot(1,2,1);
        hold on
        % Sensing patterns and cells
		for i=1:sim.N
			plot3_poly( [sim.C{i} ; zeros(size(sim.C{i}(1,:)))], 'r--');
%             plot3_poly( [sim.Cd{i} ; zeros(size(sim.Cd{i}(1,:)))], 'c--');
            if ~isempty(sim.W{i})
                plot3_poly( [sim.W{i} ; zeros(size(sim.W{i}(1,:)))], 'k');
                fill3(sim.W{i}(1,:),sim.W{i}(2,:),zeros(size(sim.W{i}(1,:))),[0.8 0.75 0.75]);
            end
		end
		
        % Node positions and cones
        for i=1:sim.N
            plot3( sim.x_quad(1,i), sim.x_quad(2,i), sim.x_quad(3,i), 'ko' )
%             quadrotor_plot(fig, [sim.x_quad(1,i), sim.x_quad(2,i), sim.x_quad(3,i)], [sim.x_quad(7,i), sim.x_quad(8,i), sim.x_quad(9,i)] );
            plot3( [sim.x_quad(1,i) sim.x_quad(1,i)], [sim.x_quad(2,i) sim.x_quad(2,i)], [sim.x_quad(3,i) 0], 'k--.' )
            if length(sim.W{i})>0
                for j=1:24:length(sim.W{i}(1,:))
                    plot3([sim.W{i}(1,j) sim.x_quad(1,i)], [sim.W{i}(2,j) sim.x_quad(2,i)], [0 sim.x_quad(3,i)], 'r--');
                end
            end
        end
        
        
        % Plot region
        plot3_poly( [sim.region ; zeros(size(sim.region(1,:)))], 'k' );
        plot3_AABB([sim.axis 0 sim.zmax], 'w.');
        grid on;
        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 max(sim.zmax)])
        axis equal
        axis off
        
        for i=1:sim.N
            numb=sprintf('#%i',i);
            text(sim.x_quad(1,i),sim.x_quad(2,i),sim.x_quad(3,i)+0.1,numb);
        end
%         text(-1,0,2,'\bf\itAltitude');
%         for i=1:sim.N
%             numb=sprintf('#%i \t %.1f%%',i,100*(sim.Z(i)-sim.zmin)/(sim.zmax-sim.zmin));
%             text(-1,0,2-(i)*0.2,numb);
%         end
%         text(-1,0,2-(sim.N+1)*0.2,'\bf\itPan');
%         for i=1:sim.N
%             numb=sprintf('#%i \t %.1f%%',i,sim.TH(i));
%             text(-1,0,2-(i+sim.N+1)*0.2,numb);
%         end
%         text(-1,0,2-(2*(sim.N+1))*0.2,'\bf\itTilt');
%         for i=1:sim.N
%             numb=sprintf('#%i \t %.1f%%',i,sim.HI(i));
%             text(-1,0,2-(i+2*sim.N+2)*0.2,numb);
%         end
%         text(-1,0,2-3*(1+sim.N)*0.2,'\bf\itZoom');
%         for i=1:sim.N
%             numb=sprintf('#%i \t %.1f%%',i,sim.D(i));
%             text(-1,0,2-(i+3*sim.N+3)*0.2,numb);
%         end


        if sim.SAVE_PLOTS
            fname = strcat( '~/Frames/', sprintf('3D_frame_%d.png', sim.s) );
            print(fname,'-dpng','-r600');
%             saveas(gcf, fname);
        else
            pause(0.01);
        end
    end
    
    

    
    
    % ----------------- Plot network phi -----------------
    if sim.PLOT_STATE_PHI
        clf
        hold on
        plot_phi( sim.phi , sim.region );
        % Region
        plot_poly( sim.region, 'k');
        % Sensing patterns and cells
        for i=1:sim.N
            plot_poly( sim.C{i}, 'r--');
            plot_poly( sim.W{i}, 'k');
        end
        % Node positions
        for i=1:sim.N
%                 tmpc = [sim.X(i) + disk_rad * cos(t) ; sim.Y(i) + disk_rad * sin(t)];
%                 fill( tmpc(1,:), tmpc(2,:), 'k', 'EdgeColor', 'none' );
            plot( sim.X(i), sim.Y(i), 'k.' )
            hold on
        end
       
        plot_AABB(sim.axis, 'w.');

        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        axis(sim.axis)
        axis equal
        axis off

        if sim.SAVE_PLOTS
            fname = strcat( '~/Frames/', sprintf('PHI_frame_%d.png', sim.s) );
            print(fname,'-dpng','-r600');
%             saveas(gcf, fname);
        else
            pause(0.01);
        end
    end
    
    
    
    % ----------------- Plot network quality -----------------
    if sim.PLOT_STATE_QUALITY
        clf
        hold on
        % Plot cylinders
        for i=1:sim.N
            plot3_cell_quality(sim.W{i}, sim.f(i), 'r');
        end
        % Plot region
        plot3_poly( [sim.region ; zeros(size(sim.region(1,:)))], 'k' );
        plot3_AABB([sim.axis 0 sim.zmax], 'w.');

        set( gca, 'Units', 'normalized', 'Position', [0 0 1 1] );
        view(-16, 34);
        axis([sim.axis 0 1])
        axis equal
        axis off

        if sim.SAVE_PLOTS
            fname = strcat( '~/Frames/', sprintf('Q_frame_%d.png', sim.s) );
            print(fname,'-dpng','-r600');
%             saveas(gcf, fname);
        else
            pause(0.01);
        end
    end
end
    