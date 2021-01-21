clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%% Set Simulation Options %%%%%%%%%%%%%%%%%%%
% Network options
% Altitude constraints
zmin = 0.3;
zmax = 3.88552;

% Simulation options
% Simulation duration in seconds
Tfinal =30;
% Time step in seconds
Tstep = 0.1;

% Control law options
% Planar control law gain
axy = 0.5;
% Altitude control law gain
az = 0.5;
% Panning control law gain
ath = 0.0005;
% Tilting control law gain
ah = 0.0005;


% Network plots to show during simulation
PLOT_STATE_2D = 0;
PLOT_STATE_3D = 1;
PLOT_STATE_QUALITY = 0;
SAVE_PLOTS = 0;

% Save simulation results to file
SAVE_RESULTS = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%------------ Set Parameters for sensing patterns ------------
h_lim=50;

%half angle of conical field of view
% delta=[deg2rad(20) deg2rad(35) deg2rad(25) deg2rad(27) deg2rad(15) deg2rad(30) deg2rad(23) deg2rad(30)];
delta=[deg2rad(20)];
delta_max=[deg2rad(35)];
delta_min=[deg2rad(20)];



% ---------------- Region ----------------
% % Bullo region
% Xb=[ -30 -10 30 10 ];
% Yb=[-10 -30 10 30];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% % Bullo region
% Xb=[ 2 4 -2 -4 ];
% Yb=[4 2 -4 -2 ];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% % Bullo region
% Xb=[ -4.414 -1.586 4.414 1.586 ];
% Yb=[-1.586 -4.414 1.586 4.414 ];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% % Bullo region
% Xb=[ -5 -1 5 1];
% Yb=[-1 -5 1 5 ];
% [Xb, Yb] = poly2cw(Xb, Yb);
% region = [Xb ; Yb];
% region_area = polyarea( Xb, Yb );
% axis_scale = [-0.5 3 -0.5 3];

% Bullo region
Xb=[ -8 2  8 -2];
Yb=[2 -8 -2 8];
[Xb, Yb] = poly2cw(Xb, Yb);
region = [Xb ; Yb];
region_area = polyarea( Xb, Yb );
axis_scale = [-0.5 3 -0.5 3];


% ---------------- Initial State ----------------
X = [0];
Y = [0 ];
Z = [1];
TH = [0];
HI=[0];

% %localization uncertainy
r=[0];
uz=[0];
% Number of nodes
N = length(X);

[am,bm,xcm,ycm] = calculate_ellipse_parameters(h_lim,0,zmax-uz,delta,X,Y,N);
dmax=bm/tan(delta);



%initial projection of field of view
[a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z,delta,X,Y,N);

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
Ctemp = cell([1 N]);
Cq = cell([1 N]);
Cz = cell([1 N]);

% Sensed space partitioning
W = cell([1 N]);

% Control inputs
uX = zeros(1,N);
uY = zeros(1,N);
uZ = zeros(1,N);
uTH = zeros(1,N);
uHI = zeros(1,N);

% Create sim struct that contains all information, used for plots
sim = struct;
sim.region = region;
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

sim.N = N;
sim.C = C;
sim.W = W;
sim.f = f;


sim.PLOT_STATE_3D = PLOT_STATE_3D;
sim.PLOT_STATE_2D = PLOT_STATE_2D;
sim.PLOT_STATE_PHI = 0;
sim.PLOT_STATE_QUALITY = PLOT_STATE_QUALITY;
sim.SAVE_PLOTS = SAVE_PLOTS;
check_resilience=0;

%%%%%%%%%%%%%%%%%%%JACOBIAN_FUNCTIONS%%%%%%%%%%%%%

% syms q xi yi zi thi ai bi xc yc h d t
% assume([q xi, yi, zi, thi,ai, bi ,xc, yc, h, d, t],'real');
% 
% % ai=(zi/2)*(tan(h+d)-tan(h-d));
% % bi=zi*tan(d)*(1+((tan(h+d)+tan(h-d))/2)^2)^0.5;
% % xc=xi+cos(t)*(zi/2)*(tan(h+d)+tan(h-d));
% % yc=yi+sin(t)*(zi/2)*(tan(h+d)+tan(h-d));
% 
% % Find value of parameter t
% q = q - [xc ; yc];
% q = ROT(q, -thi);
% t = atan2( q(2)/b, q(1)/a );
% 
% g = [a*cos(t); b*sin(t)];
% 
% C=ROT([ai*cos(t) ; bi*sin(t)]+[xc;yc],thi);
% 
% 
% Jxyi = matlabFunction([diff(C,xi)'; diff(C,yi)'],'File','J_ellipse_xy','Vars',[xi, yi, zi, thi, h, d, k]);
% Jzi = matlabFunction(diff(C,zi),'File','J_ellipse_z','Vars',[xi, yi, zi, thi, h, d, k]);
% Jthi = matlabFunction(diff(C,zi),'File','J_ellipse_th','Vars',[xi, yi, zi, thi, h, d, k]);
% Jhi = matlabFunction(diff(C,zi),'File','J_ellipse_h','Vars',[xi, yi, zi, thi, h, d, k]);

% syms b zi h d zmin zmax f b
% assume([b zi h d zmin zmax f b],'real');
% 
% b=zi*tan(d)*(1+((tan(h+d)+tan(h-d))/2)^2)^0.5;
% f=1-tanh(((b/tan(d))-zmin)/(zmax-zmin));
% 
% 
% dfz= matlabFunction(diff(f,zi),'File','dfuz','Vars',[zi h d zmin zmax]);
% 
% dfh= matlabFunction(diff(f,h),'File','dfuh','Vars',[zi h d zmin zmax]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%%%%%%
if PLOT_STATE_3D || PLOT_STATE_2D || PLOT_STATE_QUALITY
	f1=figure;
end
tic;
for s=1:smax
	fprintf('%.2f%% complete\n',100*s/smax);
    
    if check_resilience==1
        if s==smax/2
            Xs1=Xs(1:s-1,:);
            Ys1=Ys(1:s-1,:);
            Zs1=Zs(1:s-1,:);
            TH1=THs(1:s-1,:);
            HI1=HIs(1:s-1,:);
            C1=C;
            W1=W;
            yc1=ycs(1:s-1,:);
            xc1=xcs(1:s-1,:);
            as1=as(1:s-1,:);
            bs1=bs(1:s-1,:);
            [N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uz,uTH,uHI]=agent_failure(smax,N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uTH,uHI);
        end
    end
    % ----------------- Partitioning -----------------
    
    
    %calculate ellipse parameters (axis & qc)
    [a,b,xc,yc] = calculate_ellipse_parameters(HI,TH,Z-uz,delta,X,Y,N);
    %calculate ellipse parameters (axis & qc) for altitude uncertainty
    [auz,buz,xcuz,ycuz] = calculate_ellipse_parameters(HI,TH,Z+uz,delta,X,Y,N);
    % Coverage quality
    temp=[];
    for i=1:N
        temp=[temp b(i)/tan(delta(i))];
    end
    f=fu(temp, zmin, dmax);

    % Sensing patterns
    for i=1:N
        Cq{i} = ROT([(a(i)-r(i))*cos(t); (b(i)-r(i))*sin(t)],TH(i))+ ([xc(i);yc(i)]*ones(1,length(t)));
        Cz{i} = ROT([(auz(i)-r(i))*cos(t); (buz(i)-r(i))*sin(t)],TH(i))+ ([xcuz(i);ycuz(i)]*ones(1,length(t)));
        [xo,yo]=polybool('intersection',Cz{i}(1,:),Cz{i}(2,:),Cq{i}(1,:),Cq{i}(2,:));
        
        C{i}=[xo;yo];
%         if ~isempty(xo)
%             C{i}=[xo(1:length(t));yo(1:length(t))];
%         else
%             C{i}=[xo;yo];
%         end
    end

    % Store simulation data
    Xs(s,:) = X;
    Ys(s,:) = Y;
    Zs(s,:) = Z;
	THs(s,:) = TH;
    HIs(s,:) = HI;
    ycs(s,:) = yc;
    xcs(s,:) = xc;
    as(s,:) = a;
    bs(s,:) = b;

    % Sensed space partitioning
    for i=1:N
		% Find the cell of each node i based on all other nodes
		W{i} = sensed_partitioning_uniform_anisotropic_cell(region,C, f, i,N);
    end
    
    % ----------------- Plots -----------------
    sim.X = X;
    sim.Y = Y;
    sim.Z = Z;
	sim.TH = TH;
    sim.HI = HI;
    sim.a = a;
    sim.b = b;
    sim.xc = xc;
    sim.yc = yc;
    sim.C = C;
    sim.W = W;
    sim.f = f;
    sim.s = s;
    sim.N=N;
%    
%     name=sprintf('s%i.png',s);
%     saveas(gcf,name)
    clf(f1);
    plot_sim_UAV( sim);
    
    
    % ----------------- Objective -----------------
    % Find covered area and H objective
    for i=1:N
        if ~isempty(W{i})
            cov_area(s) = cov_area(s) + polyarea_nan(W{i}(1,:), W{i}(2,:));
            H(s) = H(s) + f(i) * polyarea_nan(W{i}(1,:), W{i}(2,:));
        end
    end
    
      
    
    % ----------------- Control law -----------------
    parfor i=1:N % parfor faster here
        % Create anonymous functions for the Jacobians
        % The functions include parameters specific to this node
        
			Jxy = @(q) J_ellipse_xy(q);
            
			Jz = @(q) J_ellipse_z(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a(i)-r(i), b(i)-r(i),xc(i), yc(i));
            Jzuz = @(q) J_ellipse_z(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,auz(i)-r(i), buz(i)-r(i),xcuz(i), ycuz(i));
            
			Jth = @(q) J_ellipse_th(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a(i)-r(i), b(i)-r(i),xc(i), yc(i));
            Jthuz = @(q) J_ellipse_th(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,auz(i)-r(i), buz(i)-r(i),xcuz(i), ycuz(i));
            
            Jh=@(q) J_ellipse_h(q, X(i), Y(i), Z(i)-uz(i), TH(i), HI(i) ,a(i)-r(i), b(i)-r(i),xc(i), yc(i),delta(i));
            Jhuz=@(q) J_ellipse_h(q, X(i), Y(i), Z(i)+uz(i), TH(i), HI(i) ,auz(i)-r(i), buz(i)-r(i),xcuz(i), ycuz(i),delta(i));
            
		[uX(i), uY(i)] = control_uniform_planar(region, W, C,Cq,Cz,f, i, Jxy);
		uZ(i) = control_uniform_altitude(region, W, C,Cq,Cz,f, dfuz(Z(i),HI(i),delta(i),zmin,dmax), i, Jz,Jzuz);
		uTH(i) = control_uniform_pan(region, W, C,Cq,Cz,f, i, Jth,Jthuz);
        uHI(i) = control_uniform_tilt(region, W, C,Cq,Cz,f, dfuh(Z(i),HI(i),delta(i),zmin,dmax), i, Jh,Jhuz);
    end

    
    % Control inputs with gain
    uX = axy * uX;
    uY = axy * uY;
    uZ = az * uZ;
	uTH = ath * uTH;
    uHI = ah *uHI;
    
    % ----------------- Simulate with ode -----------------
    Tspan = [s*Tstep (s+1)*Tstep];
    IC = [X Y Z TH HI]';
    u = [uX uY uZ uTH uHI]';
    [T, ode_state] = ode45(@(t,y) DYNAMICS_simple(t, y, u), Tspan, IC);
    
    % Keep the last row of XYZ
    
    Xn = ode_state(end, 1:N );
    Yn = ode_state(end, N+1:2*N );
    Zn = ode_state(end, 2*N+1:3*N );
    THn = ode_state(end, 3*N+1:4*N );
    HIn=ode_state(end, 4*N+1:5*N );
    
    parfor i=1:N
        sense=min_distance_point(Xn(i),Yn(i),Zn(i),THn(i),HIn(i),delta(i),t,r(i));
        
         if (inpolygon(Xn(i),Yn(i),Xb,Yb))
            X(i)=Xn(i);
            Y(i)=Yn(i);        
         end
        
        if (sum(inpolygon(sense(1,:),sense(2,:),Xb,Yb))>=1)
            
            if (Zn(i)<=zmax & Zn(i)>=zmin)
                Z(i)=Zn(i);   
            end
            if Zn(i)>zmax
                Z(i)=zmax;  
            end
            if Zn(i)<zmin
                Z(i)=zmin;  
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
plot( Tstep*linspace(1,s-1,s-1), 100*cov_area(1:s-1)/region_area, 'b');
axis([0 Tstep*smax 0 100]);
h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$A_{cov}~(\%)$');
set(h,'Interpreter','latex')

% Plot objective
figure;
plot( Tstep*linspace(1,s-1,s-1), H(1:s-1), 'b');
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










