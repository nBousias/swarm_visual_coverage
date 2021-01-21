clear all;
close all;
clc;



q1=load('results_uniform_anisotropic_20190314_0704.mat');
h1=[ q1.Tstep*linspace(1,q1.s,q1.s); q1.cov_area(1:q1.s)'];
close gcf;

q2=load('results_uniform_anisotropic_20190314_1148.mat');
h2=[q2.Tstep*linspace(1,q2.s,q2.s); q2.cov_area(1:q2.s)'];
close gcf;

q3=load('results_uniform_anisotropic_20190314_1209.mat');
h3=[q3.Tstep*linspace(1,q3.s,q3.s); q3.cov_area(1:q3.s)'];
close gcf;

q4=load('results_uniform_anisotropic_20190314_1429.mat');
h4=[q4.Tstep*linspace(1,q4.s,q4.s); q4.cov_area(1:q4.s)'];
close gcf;

q5=load('results_uniform_anisotropic_20190314_1434.mat');
h5=[q5.Tstep*linspace(1,q5.s,q5.s); q5.cov_area(1:q5.s)'];
close gcf;

figure;
plot(h1(1,:),h1(2,:),'y');hold on;plot(h2(1,:),h2(2,:),'k');hold on;plot(h3(1,:),h3(2,:),'r');
hold on;plot(h4(1,:),h4(2,:),'Color',[0.3 0.6 0.2]);hold on;plot(h5(1,:),h5(2,:),'b');
legend(strcat(num2str(q1.Tstep*1000),'ms'),strcat(num2str(q2.Tstep*1000),'ms')...
    ,strcat(num2str(q3.Tstep*1000),'ms'),strcat(num2str(q4.Tstep*1000),'ms'),...
    strcat(num2str(q5.Tstep*1000),'ms'));

h = xlabel('$Time ~(s)$');
set(h,'Interpreter','latex')
h = ylabel('$\mathcal{H}$');
set(h,'Interpreter','latex')