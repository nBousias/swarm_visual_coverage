function [] = quadrotor_plot( fig,p,n )
    quad.nrotors=4;
    quad.r=0.08;
    quad.d=0.16;
    hold on;   
    %figure(fig);

    a1s = zeros(1, quad.nrotors);
    b1s = zeros(1, quad.nrotors);
     % vehicle dimensons
    d = quad.d; %Hub displacement from COG
    r = quad.r; %Rotor radius

    for i = 1:quad.nrotors
        theta = (i-1)/quad.nrotors*2*pi;
        %   Di      Rotor hub displacements (1x3)
        % first rotor is on the x-axis, clockwise order looking down from above
        D(:,i) = [ d*cos(theta); d*sin(theta); 0];
        %Attitude center displacements
        C(:,i) = [ cos(theta); sin(theta); 0];
    end

        %STATE
        z = [p(1);p(2);p(3)];
        phi = n(1);    %Euler angles
        the = n(2);
        psi = n(3);
        
%         R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
%             cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
%             -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];
%         
        %Manual Construction
        R = rotation_matrix(phi,the,psi);
        
        %CALCULATE FLYER TIP POSITONS USING COORDINATE FRAME ROTATION
        F = [1 0 0;0 1 0;0 0 1];
        
        %Draw flyer rotors
        t = [0:pi/8:2*pi];
        for j = 1:length(t)
            circle(:,j) = [r*sin(t(j));r*cos(t(j));0];
        end
        
        for i = 1:quad.nrotors
            hub(:,i) = F*(z + R*D(:,i)); %points in the inertial frame  
            tippath(:,:,i) = F*R*circle;
            hold on;
            fill3([hub(1,i)+tippath(1,:,i)],[hub(2,i)+tippath(2,:,i)],[hub(3,i)+tippath(3,:,i)],[0.5 0.5 0.5]);
        end
        
        %Draw flyer
        hub0 = F*z;  % centre of vehicle
        for i = 1:quad.nrotors
            hold on;
            % line from hub to centre plot3([hub(1,N) hub(1,S)],[hub(2,N) hub(2,S)],[hub(3,N) hub(3,S)],'-b')
            plot3([hub(1,i) hub0(1)],[hub(2,i) hub0(2)],[hub(3,i) hub0(3)],'-k')
%             hold on;
             %plot a circle at the hub itself
             %plot3([hub(1,i)],[hub(2,i)],[hub(3,i)],'o')
        end
        
%         % plot the vehicle's centroid on the ground plane
%         hold on;
%         plot3([z(1) z(1)],[z(2) z(2)],[0 z(3)],'--k')
%         hold on;
%         plot3([z(1)],[z(2)],[0],'xk')
%         grid on;
    end
        