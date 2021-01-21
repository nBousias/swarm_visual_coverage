clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%% Set Simulation Options %%%%%%%%%%%%%%%%%%%
% Network options
% Altitude constraints
zmin_tot = 0.3;
zmax_tot = 1.8;

% Simulation options
% Simulation duration in seconds
Tfinal = 300;
% Time step in seconds
Tstep = 0.5;

% Control law options
% Planar control law gain
axy = 0.5;
% Altitude control law gain
az = 0.5;
% Panning control law gain
ath = 0.0005;
% Tilting control law gain
ah = 0.0005;
% Zooming control law gain
azoom = 0.0005;

% Network plots to show during simulation
PLOT_STATE_2D = 0;
PLOT_STATE_3D = 1;
PLOT_STATE_QUALITY = 0;
SAVE_PLOTS = 0;

% Save simulation results to file
SAVE_RESULTS = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%------------ Set Parameters for sensing patterns ------------


%limit tilt angle
h_lim=40;
%range of delta
delta_max=[deg2rad(30)];
delta_min=[deg2rad(15)];

% ---------------- Region ----------------
% Bullo region
Xb=[ 0, 2.125, 2.9325, 2.975, 2.9325, 2.295, 0.85, 0.17 ];
Yb=[ 0, 0, 1.5, 1.6, 1.7, 2.1, 2.3, 1.2 ];
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

% % Bullo region
% Xb=[ -5 -1 5 1];
% Yb=[-1 -5 1 5 ];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region = 10 .* [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% ---------------- Initial State ----------------
%half the angle of conic field of view
delta=[deg2rad(20) deg2rad(20) deg2rad(20) deg2rad(20) deg2rad(20) deg2rad(20)];
X = [1 2 0.5 2.2 0.8 1.7 ];
real_X=sim.X(i)+((-1)^(round(rand)))*real_uz(i)*rand(1);
Y = [1 1.8 1.5 0.3 0.5 2.1 ];
real_Y=sim.Y(i)+((-1)^(round(rand)))*real_uz(i)*rand(1);
Z = [1 1.1 0.5 0.8 1 2 ];
real_Z=sim.Z(i)+((-1)^(round(rand)))*real_uz(i)*rand(1);
TH = [0 0 0 0 0 0];
HI=[0 0 0 0 0 0];
D=delta;
% % localization uncertainy
r=[0 0 0 0 0 0];
uz=[0 0 0 0 0 0];
real_r=[0.1 0.1 0.1 0.1 0.1 0.1 ];
real_uz=[0.2 0.2 0.2 0.2 0.2 0.2];

% %half the angle of conic field of view
% delta=[deg2rad(20) deg2rad(20) deg2rad(20)];
% X = [3 4 3.5];
% Y = [3 3 3];
% Z = [1.2 0.9 1.5];
% TH = [0 0 0];
% HI=[0 0 0];
% D=delta;
% % localization uncertainy
% r=[0.1 0.1 0.1];
% uz=[0.3 0.3 0.3];

% %half the angle of conic field of view
% delta=[deg2rad(20)];
% 
% X = [1];
% Y = [1];
% Z = [1];
% TH = [0.5];
% HI=[0];
% D=delta;
% % localization uncertainy
% r=[0.1];
% uz=[0];

% delta=deg2rad(30);
% X = [0];
% Y = [0 ];
% Z = [1];
% TH = [0];
% HI=[0];
% D=delta;
% % %localization uncertainy
% r=[0];
% uz=[0];

% Number of nodes
N = length(X);

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
PPC = 120;
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
H = zeros(smax,1);

% Initialize (cell) arrays
% Coverage quality
f = zeros(1, N);

% Sensing disks
C = cell([1 N]);
Cgs = cell([1 N]);
Cd = cell([1 N]);
Ctemp = cell([1 N]);
C_maxz = cell([1 N]);
C_minz = cell([1 N]);

% Sensed space partitioning
W = cell([1 N]);

% Control inputs
uX = zeros(1,N);
uY = zeros(1,N);
uZ = zeros(1,N);
uTH = zeros(1,N);
uHI = zeros(1,N);
uD = zeros(1,N);

% Create sim struct that contains all information, used for plots
sim = struct;
sim.region = region;
sim.axis = axis_scale;
sim.PPC = PPC;
sim.zmin = zmin;
sim.zmax = zmax;
sim.X = X;
sim.real_X=real_X;
sim.Y = Y;
sim.real_Y=real_Y;
sim.Z = Z;
sim.real_Z=real_Z;
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
end
tic;
for s=1:smax
	fprintf('%.2f%% complete\n',100*s/smax);
    

    % ----------------- Partitioning -----------------
    
    %calculate ellipse parameters for D
    [a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,D,X,Y,N);
    
    %calculate ellipse parameters for D with maximum Z
    [a_maxz,b_maxz,xc_maxz,yc_maxz] = calculate_ellipse_parameters(HI,TH,Z+uz,D,X,Y,N);
        
    %calculate ellipse parameters for D with least Z
    [a_minz,b_minz,xc_minz,yc_minz] = calculate_ellipse_parameters(HI,TH,Z-uz,D,X,Y,N);
    
    %calculate ellipse parameters for delta
    [ad,bd,xcd,ycd] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,N);
    
   %calculate real ellipse parameters for delta
    [ar,br,xcr,ycr] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,N);
    
    
    % Coverage quality
    for i=1:N
		f(i)=fu(Z(i)+uz(i),HI(i),D(i), zmin(i), dmax(i),delta_min,delta_max);
    end

    % Sensing patterns
    for i=1:N
        %actual pattern
        C_r{i}=ROT([(ar(i))*cos(t); (br(i))*sin(t)],TH(i))+ ([xcr(i);ycr(i)]*ones(1,length(t)));
        %calculate pattern for new zoom angle D for Z
        C{i} = ROT([(a(i)-r(i))*cos(t); (b(i)-r(i))*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
        %calculate pattern for original zoom angle delta for Z
        Cd{i} = ROT([(ad(i)-r(i))*cos(t); (bd(i)-r(i))*sin(t)],TH(i))+ ([xcd(i);ycd(i)]*ones(1,length(t)));
        
        %for localization uncertainty
        C_maxz{i} = ROT([(a_maxz(i)-r(i))*cos(t); (b_maxz(i)-r(i))*sin(t)],TH(i))+ ([xc_maxz(i);yc_maxz(i)]*ones(1,length(t)));
        C_minz{i} = ROT([(a_minz(i)-r(i))*cos(t); (b_minz(i)-r(i))*sin(t)],TH(i))+ ([xc_minz(i);yc_minz(i)]*ones(1,length(t)));
        [xo,yo]=polybool('intersection',C_maxz{i}(1,:),C_maxz{i}(2,:),C_minz{i}(1,:),C_minz{i}(2,:));
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
    
    % ----------------- Plots -----------------
    sim.X = X;
    sim.Y = Y;
    sim.Z = Z;
	sim.TH = TH;
    sim.HI = HI;
    sim.D = D;
    sim.a = a;
    sim.b = b;
    sim.xc = xc;
    sim.yc = yc;
    sim.C = C;
    sim.Cd = Cd;
    sim.Cgs = Cgs;
    sim.C_maxz=C_maxz;
    sim.C_minz=C_minz;
    sim.W = W;
    sim.f = f;
    sim.s = s;
    sim.N=N;
%    
%     name=sprintf('s%i.png',s);
%     saveas(gcf,name)
    clf(f1);
    plot_sim_UAV_zoom(sim);
    
    
    % ----------------- Objective -----------------
    % Find covered area and H objective
    for i=1:N
        if ~isempty(W{i})
            cov_area(s) = cov_area(s) + polyarea_nan(W{i}(1,:), W{i}(2,:));
            H(s) = H(s) + f(i) * polyarea_nan(W{i}(1,:), W{i}(2,:));
        end
    end
    
         
    % ----------------- Control law -----------------
    for i=1:N % parfor faster here
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

    
    % Control inputs with gain
    uX = axy * uX;
    uY = axy * uY;
    uZ = az * uZ;
	uTH = ath * uTH;
    uHI = ah *uHI;
    uD = azoom *uD;
    
    % ----------------- Simulate with ode -----------------
    Tspan = [s*Tstep (s+1)*Tstep];
    IC = [X Y Z TH HI D]';
    u = [uX uY uZ uTH uHI uD]';
    [T, ode_state] = ode45(@(t,y) DYNAMICS_simple(t, y, u), Tspan, IC);
    
    % Keep the last row of XYZ
    
    Xn = ode_state(end, 1:N );
    Yn = ode_state(end, N+1:2*N );
    Zn = ode_state(end, 2*N+1:3*N );
    THn = ode_state(end, 3*N+1:4*N );
    HIn=ode_state(end, 4*N+1:5*N );
    Dn=ode_state(end, 5*N+1:6*N );
    
    for i=1:N
        sense=min_distance_point(Xn(i),Yn(i),Zn(i),THn(i),HIn(i),Dn(i),t,r(i));
        
        if (Dn(i)>=delta_min & Dn(i)<=delta_max)
            D(i)=Dn(i);
        else
            if Dn(i)>delta_max
                D(i)=delta_max;
            else
                D(i)=delta_min;
            end                
        end
        
         if (inpolygon(Xn(i),Yn(i),Xb,Yb))
            X(i)=Xn(i);
            Y(i)=Yn(i);        
         end
        
        if (sum(inpolygon(sense(1,:),sense(2,:),Xb,Yb))>=1)
            
            if (Zn(i)<=zmax(i) & Zn(i)>=zmin(i))
                Z(i)=Zn(i);   
            end
            if Zn(i)>zmax(i)
                Z(i)=zmax(i);  
            end
            if Zn(i)<zmin(i)
                Z(i)=zmin(i);  
            end
            
            TH(i)=THn(i);
            
            
            if (abs(HIn(i))<=deg2rad(h_lim))
               HI(i)=HIn(i); 
            end
            if HIn(i)>deg2rad(h_lim)
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

% v = VideoWriter('newfile.avi');
% open(v)
% for i=1:200
% nam=sprintf('s%i.png',i);
% yy=imread(nam);
% writeVideo(v,yy)
% end
% close(v)

% Plot covered area
figure;
plot( Tstep*linspace(1,s-1,s-1), 100*cov_area(1:s-1)/region_area, 'r--');
axis([0 Tstep*smax 0 100]);
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$A_{cov}~(\%)$');
set(h,'Interpreter','latex')

% Plot objective
figure;
plot( Tstep*linspace(1,s-1,s-1), smooth(smooth(smooth(smooth(smooth(smooth(smooth(H(1:s-1)))))))), 'k');
% hold on;
%  plot( Tstep*linspace(1,s-1,s-1), smooth(smooth(smooth(smooth(smooth(smooth(smooth(H(1:s-1)))))))), 'r--');
% legend('PTZ','no PTZ');
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$\mathcal{H}$');
set(h,'Interpreter','latex')

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










