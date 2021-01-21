
        clf
        % Sensing patterns and cells
		for i=1:3
			plot3_poly( [Cgs{i} ; zeros(size(Cgs{i}(1,:)))], 'r--');
            hold on;            
            if ~isempty(Wgs{i})
                plot3_poly( [Wgs{i} ; zeros(size(Wgs{i}(1,:)))], 'k');
                hold on;
                fill3(Wgs{i}(1,:),Wgs{i}(2,:),zeros(size(Wgs{i}(1,:))),[0.8 0.75 0.75]);
                hold on;
            end
        end
		
        
        X=[0.3664    0.6355    1.2121];
        Y=[0.3181    1.1472    0.3813];
        Z=[1.4119    1.4223    1.4271];
        
        for i=1:3

            plot3( posx(:,i), posy(:,i), posz(:,i), 'b' )
            hold on;
            plot3_poly( [W{i} ; zeros(size(W{i}(1,:)))], '--k');
            hold on;
            numb=sprintf('#%i',i);
            text(X(i),Y(i),Z(i)+0.1,numb);
        end
        
        % Node positions and cones
        for i=1:3   
            hold on;
            plot3( X(i), Y(i), Z(i), 'ko' );
            hold on;
            plot3( [X(i) X(i)], [Y(i) Y(i)], [Z(i) 0], 'k--.' )
            if length(Wgs{i})>0
                for j=1:24:length(Wgs{i}(1,:))
                    hold on;
                    plot3([Wgs{i}(1,j) X(i)], [Wgs{i}(2,j) Y(i)], [0 Z(i)], 'r--');
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
        
        


        
        
        