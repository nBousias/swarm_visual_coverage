clear all;
close all;
clc;

% mkdir video1
% video_name='RESILIENCE.avi'

%%%%%%%%%%%%%%%%%%% Set Simulation Options %%%%%%%%%%%%%%%%%%%
% Network options
% Altitude constraints
zmin_tot = 0.3;
zmax_tot = 2.3;

% Simulation options
% Simulation duration in seconds
Tfinal = 150;
% Time step in seconds
Tstep = 0.1;

% Control law options
% Planar control law gain
Kx = 0.25;
Ky = 0.25;
% Altitude control law gain
Kz = 0.25;
% Panning control law gain
Kth = 0.0005;
% Tilting control law gain
Kh = 0.0005;
% Zooming control law gain
Kzoom = 0.0005;

% Network plots to show during simulation
PLOT_STATE_2D = 0;
PLOT_STATE_3D = 1;
PLOT_STATE_QUALITY = 0;
SAVE_PLOTS = 0;

% Save simulation results to file
SAVE_RESULTS = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%------------ Set Parameters for sensing patterns ------------


% %range of delta
delta_max=[deg2rad(30)];
delta_min=[deg2rad(15)];

%limit tilt angle
h_lim=30;


% ---------------- Region ----------------

% %   Non-convex region
% Xb=[ 0 0 5 5 2.5 2.5 5 5];
% Yb=[ 0 5 5 3.5 3.5 1.5 1.5 0];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% %   Non-convex region
% Xb=[0 0 1.2 1.2 3 3 4 4 3.5 3.5 1 1];
% Yb=[0 2 2 1.5 1.5 3 3 2 2 1 1 0];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% Bullo region
Xb=[ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ]-1;
Yb=[ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ]-1;
[Xb, Yb] = poly2cw(Xb, Yb);
region = [Xb ; Yb];
region_area = polyarea( Xb, Yb );
axis_scale = [-0.5 3 -0.5 3];


% % Bullo region
% Xb=[ 2 4 -2 -4 ];
% Yb=[4 2 -4 -2 ];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% ---------------- Initial State ----------------
N = 6; % Number of nodes
dropout=zeros(1,N);
%half the angle of conic field of view
delta=ones(1,N)*deg2rad(20);
X = [1 2 0.5 -2.2 0.8 1.7]/3;
Y = [1 1.8 1.5 0.3 -0.5 -2.1]/2;
Z = [1 1.1 0.5 0.8 1 2];
TH = zeros(1,N);
HI = zeros(1,N);
% localization uncertainy
r = 0.05*ones(1,N);
uz = 0.05*ones(1,N);
D = delta;

zmin=zmin_tot+uz;
zmax=zmax_tot-uz;

%initial projection of field of view
[a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,D,X,Y,N);

%maximum distance of MAA from center of sensing pattern
[am,bm,xcm,ycm] = calculate_ellipse_parameters(h_lim*ones(1,N),0*ones(1,N),zmax,delta_max*ones(1,N),X,Y,N);
dmax=bm/tan(delta_max);

% ---------------- Simulation initializations ----------------

% Simulation steps
smax = floor(Tfinal/Tstep);
% Points Per Circle
PPC = 250;
% Radius for points on plots
disk_rad = 0.02;
% Vector for circle parameterization
t = linspace(0, 2*pi, PPC+1);
t = t(1:end-1); % remove duplicate last element
t = fliplr(t); % flip to create CW ordered circles


% Simulation data storage
Xs = zeros(smax, N);
Ys = zeros(smax, N);
Zs = zeros(smax, N);
THs = zeros(smax, N);
HIs = zeros(smax, N);
Ds = zeros(smax, N);

ycs = zeros(smax, N);
xcs = zeros(smax, N);
as = zeros(smax, N);
bs = zeros(smax, N);

cov_area = zeros(smax,1);
% cov_arear = zeros(smax,1);
H = zeros(smax,1);
% Hr = zeros(smax,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------quadcopter setup-------------------------------------
K_quad=load('K_quad.mat');

% Model Parameters
m_quad           = 0.030;              % mass of quadrotor (kg)
L_quad           = 0.046;           % length from center of mass to point of thrust (meters)
J_quad           = zeros(3,3);     % moments of inertia in (kg*m^2)
J_quad(1,1)      = 1.43e-5;
J_quad(2,2)      = 1.43e-5;
J_quad(3,3)      = 2.89e-5;
g                = 9.81;           % gravity (m/s^2)
quad_states      = 12;             % number of states
dt               = 1/500;
rotor_speed_min  = 0;       %rad/s
rotor_speed_max  = 2500;
K_F              = 2.3e-08; % N/(rad/s)**2
K_M              = 7.8e-11; % Nm/(rad/s)**2
gamma            = K_M/K_F;


% initial state configurations
% [x y z x_dot y_dot z_dot phi theta psi phi_dot theta_dot psi_dot]'
x_quad=[X;Y;Z;zeros(quad_states-3,N)] ;
x0=x_quad;

sim = struct;
sim.m          = m_quad;
sim.L          = L_quad;
sim.J          = J_quad;
sim.inv_inertia= inv(J_quad);
sim.gr         = g;
sim.states     = quad_states;
sim.dt         = dt;
sim.x0         = x0;
sim.x_quad     = x_quad;
sim.quad_control_gain= K_quad;
sim.Kp = diag([5;5;5]); 
sim.Kd = 2*sqrt(sim.Kp);
sim.Kr = diag([550;550;550]);
sim.Kw = diag([75;75;75]);
sim.K_F        =K_F;
sim.K_M        =K_M;
sim.gamma      =gamma;
sim.rotor_speed_min=rotor_speed_min;
sim.rotor_speed_max=rotor_speed_max;
sim.F2U        = [1 1 1 1;
                  0 L_quad 0 -L_quad
                  -L_quad 0 L_quad 0
                  gamma -gamma gamma -gamma];
sim.U2F        = inv(sim.F2U);



X_target = zeros(smax, N);
Y_target = zeros(smax, N);
Z_target = zeros(smax, N);

e_x=zeros(smax,N);
e_y=zeros(smax,N);
e_z=zeros(smax,N);


%----------------------------------------------------------------------------------
%----------------------------------------------------------------------------------

% Initialize (cell) arrays
% Coverage quality
f = zeros(1, N);
% fr = zeros(1, N);

% Sensing disks
C = cell([1 N]);
% Cr = cell([1 N]);
Cgs = cell([1 N]);
Cd = cell([1 N]);
Ctemp = cell([1 N]);
C_maxz = cell([1 N]);
C_minz = cell([1 N]);

% Sensed space partitioning
W = cell([1 N]);
% Wr = cell([1 N]);
% Control inputs
uX = zeros(1,N);
uY = zeros(1,N);
uZ = zeros(1,N);
uTH = zeros(1,N);
uHI = zeros(1,N);
uD = zeros(1,N);

% Create sim struct that contains all information, used for plots
sim.region = region;
sim.axis = axis_scale;
sim.PPC = PPC;
sim.zmin = zmin;
sim.zmax = zmax;
sim.X = X;
sim.Y = Y;
sim.Z = Z;
% sim.theta=theta;
% sim.phi=phi;
% sim.psi=psi;
% sim.real_X = real_X;
% sim.real_Y = real_Y;
% sim.real_Z = real_Z;
sim.TH = TH;
sim.HI = HI;
sim.D = D;
sim.delta=delta;
sim.a = a;
sim.b = b;
sim.xc = xc;
sim.yc = yc;

sim.dmax=dmax;
sim.N = N;
sim.C = C;
sim.Cd = Cd;
sim.C_maxz=C_maxz;
sim.C_minz=C_minz;
sim.Cgs = Cgs;
sim.W = W;
sim.f = f;

sim.PLOT_STATE_3D = PLOT_STATE_3D;
sim.PLOT_STATE_2D = PLOT_STATE_2D;
sim.PLOT_STATE_PHI = 0;
sim.PLOT_STATE_QUALITY = PLOT_STATE_QUALITY;
sim.SAVE_PLOTS = SAVE_PLOTS;
check_resilience=0;

%%%%%%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%%%%%%
if PLOT_STATE_3D || PLOT_STATE_2D || PLOT_STATE_QUALITY
	f1=figure;
%     f2=figure;
end

tic;


for s=1:smax
	fprintf('%.2f%% complete\n',100*s/smax);
    
%     if check_resilience==1
%         if s==smax/2
%             Xs1=Xs(1:s-1,:);
%             Ys1=Ys(1:s-1,:);
%             Zs1=Zs(1:s-1,:);
%             TH1=THs(1:s-1,:);
%             HI1=HIs(1:s-1,:);
%             C1=C;
%             W1=W;
%             yc1=ycs(1:s-1,:);
%             xc1=xcs(1:s-1,:);
%             as1=as(1:s-1,:);
%             bs1=bs(1:s-1,:);
%             
%             ycds1= ycds(1:s-1,:);
%             xcds1= xcds(1:s-1,:);
%             ads1= ads(1:s-1,:);
%             bds1= bds(1:s-1,:);
%             
%             [N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uz,uTH,uHI,uD,xcd,ycd,ad,bd,ycds,xcds,ads,bds,D,Ds]=agent_failure(smax,N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uz,uTH,uHI,uD,xcd,ycd,ad,bd,ycds,xcds,ads,bds,D,Ds);
%         end
%     end
    % ----------------- Partitioning -----------------
    
    %calculate ellipse parameters for D
    [a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,D,X,Y,N);
    
    %calculate ellipse parameters for D with maximum Z
    [a_maxz,b_maxz,xc_maxz,yc_maxz] = calculate_ellipse_parameters(HI,TH,Z+uz,D,X,Y,N);
        
    %calculate ellipse parameters for D with least Z
    [a_minz,b_minz,xc_minz,yc_minz] = calculate_ellipse_parameters(HI,TH,Z-uz,D,X,Y,N);
    
    %calculate ellipse parameters for delta
    [ad,bd,xcd,ycd] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,N);
    
    
    % Coverage quality
    for i=1:N
		f(i)=fu(Z(i)+uz(i),HI(i),D(i), zmin(i), dmax(i),delta_min,delta_max);
    end

    % Sensing patterns
    parfor i=1:N
        %calculate pattern for new zoom angle D for Z
        C{i} = ROT([(a(i)-r(i))*cos(t); (b(i)-r(i))*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
        
%         %calculate pattern for real position
%         Cr{i} = ROT([(ar(i))*cos(t); (br(i))*sin(t)],TH(i))+ ([xcr(i);ycr(i)]*ones(1,length(t)));
        
        %calculate pattern for original zoom angle delta for Z
        Cd{i} = ROT([(ad(i)-r(i))*cos(t); (bd(i)-r(i))*sin(t)],TH(i))+ ([xcd(i);ycd(i)]*ones(1,length(t)));
        
        %for localization uncertainty
        C_maxz{i} = ROT([(a_maxz(i)-r(i))*cos(t); (b_maxz(i)-r(i))*sin(t)],TH(i))+ ([xc_maxz(i);yc_maxz(i)]*ones(1,length(t)));
        C_minz{i} = ROT([(a_minz(i)-r(i))*cos(t); (b_minz(i)-r(i))*sin(t)],TH(i))+ ([xc_minz(i);yc_minz(i)]*ones(1,length(t)));
        [xo,yo]=polybool('intersection',C_maxz{i}(1,:),C_maxz{i}(2,:),C_minz{i}(1,:),C_minz{i}(2,:));
        [xo,yo]=polybool('intersection',xo,yo,Xb,Yb);
        Cgs{i}=[xo;yo];
%         if ~isempty(xo)
%             Cgs{i}=[xo(1:length(t));yo(1:length(t))];
%         else
%             Cgs{i}=[xo;yo];
%         end
        
        
    end

    % Store simulation data
    Xs(s,:) = X;
    Ys(s,:) = Y;
    Zs(s,:) = Z;
%     real_Xs(s,:) = real_X;
%     real_Ys(s,:) = real_Y;
%     real_Zs(s,:) = real_Z;
	THs(s,:) = TH;
    HIs(s,:) = HI;
    Ds(s,:) = D;
    ycs(s,:) = yc;
    xcs(s,:) = xc;
    as(s,:) = a;
    bs(s,:) = b;
    ycds(s,:) = ycd;
    xcds(s,:) = xcd;
    ads(s,:) = ad;
    bds(s,:) = bd;
    
    % Sensed space partitioning
    for i=1:N
		% Find the cell of each node i based on all other nodes
		W{i} = sensed_partitioning_uniform_anisotropic_cell(region,Cgs, f, i,N);
    end
    
%     for i=1:N
% 		% Find the cell of each node i based on all other nodes 
%         %for real position
%         fr(i)=fu(real_Z(i),HI(i),D(i), zmin(i), dmax(i),delta_min,delta_max);
% 		Wr{i} = sensed_partitioning_uniform_anisotropic_cell(region,Cr, fr, i,N);
%     end
    
    % ----------------- Plots -----------------
    sim.X = X;
    sim.Y = Y;
    sim.Z = Z;
%     sim.real_X = real_X;
%     sim.real_Y = real_Y;
%     sim.real_Z = real_Z;
%     sim.ar = ar;
%     sim.br = br;
%     sim.xcr = xcr;
%     sim.ycr = ycr;
	sim.TH = TH;
    sim.HI = HI;
    sim.D = D;
    sim.a = a;
    sim.b = b;
    sim.xc = xc;
    sim.yc = yc;
    sim.C = C;
%     sim.Cr = Cr;
    sim.Cd = Cd;
    sim.Cgs = Cgs;
    sim.C_maxz=C_maxz;
    sim.C_minz=C_minz;
    sim.W = W;
    sim.f = f;
%     sim.Wr = Wr;
%     sim.fr = fr;
    sim.s = s;
    sim.N = N;
    sim.x_quad = x_quad;
%    
%     name=sprintf('s%i.png',s);
%     saveas(gcf,name)
%     clf(f2);
%     plot( Tstep*linspace(1,s,s), H(1:s), 'k');
    clf(f1);
    plot_sim_UAV_zoom(f1,sim);
% progress= sprintf('%.2f sec\n%.2f%%',s*Tstep,s*100/smax);
% text(4,4,progress)
% % waitbar(s / smax,f1)
% NAME= sprintf('video1/s%d.png',s);
% saveas(gcf,NAME);
%     
    % ----------------- Objective -----------------
    % Find covered area and H objective
    for i=1:N
        if ~isempty(W{i})
            cov_area(s) = cov_area(s) + polyarea_nan(W{i}(1,:), W{i}(2,:));
            H(s) = H(s) + f(i) * polyarea_nan(W{i}(1,:), W{i}(2,:));
        end
    end
%     % ----------------- Real Objective -----------------
%     % Find covered area and H objective
%     for i=1:N
%         if ~isempty(Wr{i})
%             cov_arear(s) = cov_arear(s) + polyarea_nan(Wr{i}(1,:), Wr{i}(2,:));
%             Hr(s) = Hr(s) + fr(i) * polyarea_nan(Wr{i}(1,:), Wr{i}(2,:));
%         end
%     end
        
         
    % ---------------- Control law -----------------
    
    % ----------------- Control law -----------------
    parfor i=1:N % parfor faster here
        % Create anonymous functions for the Jacobians
        % The functions include parameters specific to this node
        
        Jxy = @(q) J_ellipse_xy(q);
        %jacobians for z_min possible      
			Jz_maxz = @(q) J_ellipse_z(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,a_maxz(i)-r(i), b_maxz(i)-r(i),xc_maxz(i), yc_maxz(i));
            
			Jth_maxz = @(q) J_ellipse_th(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,a_maxz(i)-r(i), b_maxz(i)-r(i),xc_maxz(i), yc_maxz(i));
            
            Jh_maxz=@(q) J_ellipse_h(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,a_maxz(i)-r(i), b_maxz(i)-r(i),xc_maxz(i), yc_maxz(i),D(i));
            
            Jd_maxz=@(q) J_ellipse_d(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,a_maxz(i)-r(i), b_maxz(i)-r(i),xc_maxz(i), yc_maxz(i),D(i));
        %jacobians for z_max possible
            Jz_minz = @(q) J_ellipse_z(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a_minz(i)-r(i), b_minz(i)-r(i),xc_minz(i), yc_minz(i));
            
			Jth_minz = @(q) J_ellipse_th(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a_minz(i)-r(i), b_minz(i)-r(i),xc_minz(i), yc_minz(i));
            
            Jh_minz=@(q) J_ellipse_h(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a_minz(i)-r(i), b_minz(i)-r(i),xc_minz(i), yc_minz(i),D(i));
            
            Jd_minz=@(q) J_ellipse_d(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a_minz(i)-r(i), b_minz(i)-r(i),xc_minz(i), yc_minz(i),D(i));
        
            %control laws
		[uX(i), uY(i)] = control_uniform_planar(region, W, Cgs,f, i, Jxy);
		uZ(i) = control_uniform_altitude(region, W, Cgs,C_maxz,f, dfuz(Z(i)+uz(i),HI(i),D(i),zmin(i),dmax(i),delta_min,delta_max), i, Jz_maxz,Jz_minz);
		uTH(i) = control_uniform_pan(region, W, Cgs,C_maxz,f, i, Jth_maxz,Jth_minz);
        uHI(i) = control_uniform_tilt(region, W, Cgs,C_maxz,f, dfuh(Z(i)+uz(i),HI(i),D(i),zmin(i),dmax(i),delta_min,delta_max), i, Jh_maxz,Jh_minz);
        uD(i) = control_uniform_zoom(region, W, Cgs,C_maxz,f, dfud(Z(i)+uz(i),HI(i),D(i),zmin(i),dmax(i),delta_min,delta_max), i, Jd_maxz,Jd_minz);

    end
    
%     %adaptive gains for trajectory generator
%     lamda=5;
%     if s>1
%         Kx=Kx0*2/(exp(lamda*max(abs(e_x(s,:))))+1);
%         Ky=Ky0*2/(exp(lamda*max(abs(e_y(s,:))))+1);
%         Kz=Kz0*2/(exp(lamda*max(abs(e_z(s,:))))+1);
%     end
    
    % Control inputs with gain
    uX = Kx * uX;
    uY = Ky * uY;
    uZ = Kz * uZ;
	uTH = Kth * uTH;
    uHI = Kh *uHI;
    uD = Kzoom *uD;
    
    % ----------------- Simulate with ode -----------------
    Tspan = [s*Tstep (s+1)*Tstep];
    IC = [X Y Z TH HI D]';
    u = [uX uY uZ uTH uHI uD]';
    [T, ode_state] = ode45(@(t,y) DYNAMICS_simple(t, y, u), Tspan, IC);
    
    % Keep the last row of XYZTHD
    X_target(s,:) = ode_state(end, 1:N );
    Y_target(s,:) = ode_state(end, N+1:2*N );
    Z_target(s,:) = ode_state(end, 2*N+1:3*N );
    THnn = ode_state(end, 3*N+1:4*N );
    HInn=ode_state(end, 4*N+1:5*N );
    Dnn=ode_state(end, 5*N+1:6*N );
    


    %--------------- insert quadrotor model ------------------

    
    for node=1:N
        x_t = [X_target(s,node); Y_target(s,node); Z_target(s,node)];
        coef = coeff(x_quad(1:3,node),x_t,Tstep);
        
        for time=0:dt:Tstep
            u_quad = quad_controller(sim,x_quad(:,node),coef,time);
            dF = Quadrotor_Dynamics(sim,x_quad(:,node),u_quad);
            x_quad(:,node) = x_quad(:,node)+ dF*dt;

        end
        
    end
    
    e_x(s,:) = X_target(s,:)-x_quad(1,:);
    e_y(s,:) = Y_target(s,:)-x_quad(2,:);
    e_z(s,:) = Z_target(s,:)-x_quad(3,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     x_quad(1,:) = X_target(s,:);
%     x_quad(2,:) = Y_target(s,:);
%     x_quad(3,:) = Z_target(s,:);
%     
    for i=1:N
        sense=min_distance_point(x_quad(1,i),x_quad(2,i),x_quad(3,i),THnn(i),HInn(i),Dnn(i),t,r(i));
        
        if (Dnn(i)>=delta_min & Dnn(i)<=delta_max)
            D(i)=Dnn(i);
        else
            if Dnn(i)>delta_max
                D(i)=delta_max;
            else
                D(i)=delta_min;
            end                
        end
        
%          if (inpolygon(x_quad(1,i),x_quad(2,i),Xb,Yb))
%             X(i)=x_quad(1,i);
%             Y(i)=x_quad(2,i);  
% %             
% %             real_X(i) = Xn(i)+((-1)^(round(rand)))*real_r(i)*rand(1);
% %             real_Y(i) = Yn(i)+((-1)^(round(rand)))*real_r(i)*rand(1);
%          end
            X(i)=x_quad(1,i);
            Y(i)=x_quad(2,i); 
            
        if (sum(inpolygon(sense(1,:),sense(2,:),Xb,Yb))>=1)
            
            if (x_quad(3,i)<=zmax(i) & x_quad(3,i)>=zmin(i))
                Z(i)=x_quad(3,i);   
            end
            if x_quad(3,i)>zmax(i)
                Z(i)=zmax(i);  
            end
            if x_quad(3,i)<zmin(i)
                Z(i)=zmin(i);  
            end
%             real_Z(i) = Z(i)+((-1)^(round(rand)))*real_r(i)*rand(1);
             
            TH(i)=THnn(i);

            if (abs(HInn(i))<=deg2rad(h_lim))
               HI(i)=HInn(i); 
            end
            if HInn(i)>deg2rad(h_lim)
               HI(i)=deg2rad(h_lim);
            end
            
        end
        
    end
    
    end

elapsed_time = toc;
average_iteration = elapsed_time / smax;
fprintf('\nSimulation time: %.4f s\n', elapsed_time)
fprintf('Average iteration time: %.4f s\n', average_iteration)



%%%%%%%%%%%%%%%%%%% Final plots %%%%%%%%%%%%%%%%%%%
hold on;
plot3(Xs,Ys,Zs,'b');

% plot(Xs,Ys,'b');

% v = VideoWriter(video_name);
% v.FrameRate = 4/Tstep;
% open(v)
% for i=1:smax
% nam=sprintf('video1/s%i.png',i);
% yy=imread(nam);
% writeVideo(v,yy)
% end
% close(v)

% Plot covered area
figure;
plot( Tstep*linspace(1,s-1,s-1), 100*cov_area(1:s-1)/region_area, 'r');
% plot( Tstep*linspace(1,s,s), 100*cov_area(1:s)/region_area, 'r--');
% legend('virtual locations','real locations');
axis([0 Tstep*smax 0 100]);
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$A_{cov}~(\%)$');
set(h,'Interpreter','latex')


% Plot objective
figure;
%hold on;
plot( Tstep*linspace(1,s-1,s-1), H(1:s-1), 'r');
%plot( Tstep*linspace(1,s-1,s-1), H(1:s-1),'Color', [0.3 0.6 0.2]);
% hold on;
% plot( Tstep*linspace(1,s,s), H(1:s), 'r');
% legend('Voronoi-free','Voronoi');
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$\mathcal{H}$');
set(h,'Interpreter','latex')


figure;
subplot(3,1,1);
plot( Tstep*linspace(1,s,s), e_x);
grid on;
title('Position Errors $x_{quad}-\hat{x}$','Interpreter','latex');
ylabel('$e_{x}$','Interpreter','latex');
subplot(3,1,2);
plot( Tstep*linspace(1,s,s), e_y);
grid on;
ylabel('$e_{y}$','Interpreter','latex');
subplot(3,1,3);
plot( Tstep*linspace(1,s,s), e_z);
grid on;
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
ylabel('$e_{z}$','Interpreter','latex');


figure;
subplot(3,1,1);
plot( Tstep*linspace(1,s,s), Xs);
grid on;
title('Position of quadrotor','Interpreter','latex');
ylabel('$x_{quad}(t)$','Interpreter','latex');
subplot(3,1,2);
plot( Tstep*linspace(1,s,s), Ys);
grid on;
ylabel('$y_{quad}(t)$','Interpreter','latex');
subplot(3,1,3);
plot( Tstep*linspace(1,s,s), Zs);
grid on;
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
ylabel('$z_{quad}(t)$','Interpreter','latex');

% figure;
% sim.X=sim.real_X;
% sim.Y=sim.real_Y;
% sim.Z=sim.real_Z;
% sim.C=sim.Cr;
% sim.W=sim.Wr;
% sim.a = sim.ar;
% sim.b =sim.br;
% sim.xc =sim.xcr;
% sim.yc =sim.ycr;
% plot_sim_UAV_zoom(sim);

% Save trajectories
traj = zeros(5,smax,N);
traj(1,:,:) = Xs;
traj(2,:,:) = Ys;
traj(3,:,:) = Zs;
traj(4,:,:) = THs;
traj(5,:,:) = HIs;
%%%%%%%%%%%%%%%%%%% Save Results %%%%%%%%%%%%%%%%%%%
if SAVE_RESULTS
    filename = ...
        strcat( 'results_uniform_anisotropic_', ...
        datestr(clock,'yyyymmdd_HHMM') , '.mat' );
    save(filename);
end






