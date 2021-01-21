clear all;
close all;
clc;

mkdir video1
video_name='N3_old.avi';
data_name='C:\Users\strulab\Desktop\MMA_area_coverage\paper2\results_uniform_anisotropic_20180922_1927.mat';
load(data_name);


f1=figure;
% h=waitbar(0,'test','windowstyle', 'modal');
% frames = java.awt.Frame.getFrames();
% frames(end).setAlwaysOnTop(1); 
% % The child of the waitbar is an axes object.  Grab the axes 
% % object 'c' and set its parent to be your figure f so that it now 
% % resides on figure f rather than on the old default figure. 
% c = get(h,'Children'); 
% set(c,'Parent',f1);  % Set the position of the WAITBAR on your figure 
% set(c,'Units','Normalized','Position',[.5 .5 .3 .05]);  
% % Close the default figure 
% close(h);  


for s=1:smax
    
sim.X=Xs(s,:);
sim.Y=Ys(s,:);
sim.Z=Zs(s,:);
sim.TH = THs(s,:);
sim.HI = HIs(s,:);
sim.D = Ds(s,:);

%calculate ellipse parameters for D with maximum Z
    [a_maxz,b_maxz,xc_maxz,yc_maxz] = calculate_ellipse_parameters(sim.HI,sim.TH,sim.Z+uz,sim.D,sim.X,sim.Y,sim.N);   
    %calculate ellipse parameters for D with least Z
    [a_minz,b_minz,xc_minz,yc_minz] = calculate_ellipse_parameters(sim.HI,sim.TH,sim.Z-uz,sim.D,sim.X,sim.Y,sim.N);
% Coverage quality
    for i=1:sim.N
		f(i)=fu(sim.Z(i)+uz(i),sim.HI(i),sim.D(i), zmin(i), dmax(i),delta_min,delta_max);
    end

    % Sensing patterns
    parfor i=1:N
        %for localization uncertainty
        C_maxz{i} = ROT([(a_maxz(i)-r(i))*cos(t); (b_maxz(i)-r(i))*sin(t)],sim.TH(i))+ ([xc_maxz(i);yc_maxz(i)]*ones(1,length(t)));
        C_minz{i} = ROT([(a_minz(i)-r(i))*cos(t); (b_minz(i)-r(i))*sin(t)],sim.TH(i))+ ([xc_minz(i);yc_minz(i)]*ones(1,length(t)));
        [xo,yo]=polybool('intersection',C_maxz{i}(1,:),C_maxz{i}(2,:),C_minz{i}(1,:),C_minz{i}(2,:));
        [xo,yo]=polybool('intersection',xo,yo,Xb,Yb);
        Cgs{i}=[xo;yo];
       
    end
        
    % Sensed space partitioning
    for i=1:sim.N
		% Find the cell of each node i based on all other nodes
		W{i} = sensed_partitioning_uniform_anisotropic_cell(region,Cgs, f, i,sim.N);
    end

sim.Cgs = Cgs;
sim.C=Cgs;
sim.W = W;
sim.f = f;
sim.s = s;
sim.N=N; 

clf(f1);
plot_sim_UAV_zoom(f1,sim);
progress= sprintf('%.2f sec\n%.2f%%',s*Tstep,s*100/smax);
text(4,4,progress)
% waitbar(s / smax,f1)
NAME= sprintf('video1/s%d.png',s);
saveas(gcf,NAME);
%     
end 
 

v = VideoWriter(video_name);
v.FrameRate = 1/Tstep;
open(v)
for i=1:s
nam=sprintf('video1/s%i.png',i);
yy=imread(nam);
writeVideo(v,yy)
end
close(v)
    
    