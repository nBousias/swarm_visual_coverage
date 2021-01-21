
%=== Design State Feedback for Pole Clustering within an LMI Region of an uncertain Polytopic System
disp('Design State Feedback for Pole Clustering within an LMI Region of an Uncertain Polytopic System ')
disp('Applied to the Linearized Quadrotor provided by NB and assuming that the DA and DB are the Vertices of the Polytope ')
clc; 
clear all; 
close all

p=genpath('YALMIP-master')
addpath(p)
% addpath('C:\Users\nbous\Desktop\Collaborative_Coverage\MMA_area_coverage\quadrotor collaborative\CODE\YALMIP-master\YALMIP-master\@sdpvar')
% %======================================================================= 
% % step#0   LMI Region pole clustering                                  
% %=======================================================================
% disp('  ***    Chose/Define LMI region:  alpha, r, zeta=cos(theta)  **** ')
% %======= LMI REGION#0 = NON-AGGRESSIVE = {alpha = 0.25, r = 6, 
% area_num=0; alpha = 0.0 ; % <======  CHANGE THAT  !!!!!!!!!!!!!!!!!
% %======= LMI REGION#3 = MILDLY AGGRESSIVE = {alpha = 3, r = 12, theta_deg   = 40}  
% area_num=3; alpha = 3.0    % <======  CHANGE THAT  !!!!!!!!!!!!!!!!!
% %======= LMI REGION#5 = MILDLY AGGRESSIVE = {alpha = 3, r = 12, theta_deg   = 40}  
% area_num=5; alpha = 5.0    % <======  CHANGE THAT  !!!!!!!!!!!!!!!!!
% %======= LMI REGION#6 = AGGRESSIVE = {alpha = 6, r = 12, theta_deg = 40}  
% area_num=6; alpha = 6.0   % <======  CHANGE THAT  !!!!!!!!!!!!!!!!!
% %======= LMI REGION#9 = AGGRESSIVE = {alpha = 9, r = 12, theta_deg   = 40}
% area_num=9; alpha = 9.0   % <======  CHANGE THAT  !!!!!!!!!!!!!!!!!

%========  FIX   r=12, theta = 40 deg
disp('FIX   r=12, theta = 40 deg')
r = 12 ; theta_deg   = 40 
theta = theta_deg

%======================================================================= 
% step#1   Read the Nominal Pair {Anom,Bnom} and the FOUR Uncertain Pairs {DA,DB}                               
%=======================================================================
load system_matrices.mat

%==== extract the nominal {A,Bu} matrix pair
disp('extract the nominal {A,Bu} matrix pair')
Anom=A0 ; %R12x12
Bnom=B0 ; %R12x4

%https://www.mathworks.com/help/matlab/matlab_prog/access-data-in-a-cell-array.html
disp('extract the four A-vertices {Ai, i=1,2,3,4} from DA cell using Curly Braces, {}')
A1=DA{1,1}; A2=DA{1,2}; A3=DA{1,3}; A4=DA{1,4};
disp('extract the four B-vertices {Bi, i=1,2,3,4} from DB cell using Curly Braces, {}')
B1=DB{1,1}; B2=DB{1,2}; B3=DB{1,3}; B4=DB{1,4};
%https://www.mathworks.com/help/matlab/matlab_prog/access-data-in-a-cell-array.html

%======================================================================= 
% step#2 Create the four Vertex Pairs {Avertex,Bvertex} nedded for the Polytopc Version of Quadratic D-Stabilization                               
%=======================================================================
disp('step#2 Create the four Vertex Pairs {Avertex,Bvertex} nedded for the Polytopc Version of Quadratic D-Stabilization ')
Avert1=Anom+A1 ; Bvert1=Bnom+B1;
Avert2=Anom+A2 ; Bvert2=Bnom+B2;
Avert3=Anom+A3 ; Bvert3=Bnom+B3;
Avert4=Anom+A4 ; Bvert4=Bnom+B4;

%============================================================
%  step#4   Define & Solve the LMI problem
%============================================================
disp('step#4  Formulate & Solve the LMI Quad-Hurwitz problem on the four Vertex Pairs {Avertex,Bvertex} using common {X,Y} decision variables  ')

nx = size(Anom,1);         % number of states
% [nz nw] = size(Dzw);    % number of disturbances
nu = size(Bnom,2);
% Define optimization variables
X    = sdpvar(nx,nx,'symmetric');
Y    = sdpvar(nu,nx,'full');

QuadHurwitzSynth1 = Avert1*X + X'*Avert1'  + Bvert1*Y  + Y'*Bvert1' ;   %kron(L,X) + kron(M,Avert1*X) + kron(M',(Avert1*X)')
QuadHurwitzSynth2 = Avert2*X + X'*Avert2'  + Bvert2*Y  + Y'*Bvert2' ;   %kron(L,X) + kron(M,Avert2*X) + kron(M',(Avert2*X)')
QuadHurwitzSynth3 = Avert3*X + X'*Avert3'  + Bvert3*Y  + Y'*Bvert3' ; 
QuadHurwitzSynth4 = Avert4*X + X'*Avert4'  + Bvert4*Y  + Y'*Bvert4' ; 

%== Combine
QuadHurwitzTotal = [QuadHurwitzSynth1<0, QuadHurwitzSynth2<0, QuadHurwitzSynth3<0,QuadHurwitzSynth4<0,  X>0];

%===================================================================
% Solve LMI problem
%===================================================================
% Set solver options
settings                = sdpsettings;
settings.solver         = 'lmilab';
settings.lmilab.reltol  =  0.0001;
settings.lmilab.maxiter =  1e3;
settings.lmilab.L       =  1e3; % ????
settings.verbose        =  1;
settings.showprogress   =  1;
settings.debug =1 ;

%=== Solve system of LMI constraints
%  optimize is the common function for solving optimization problems: diagnostics = optimize(Constraints,Objective,options)
Diagnostic  = optimize(QuadHurwitzTotal,[],settings);

%== Obtain LMI variables
X_sol = double(X)
Y_sol = double(Y);
%===================================================================
% Output variables
%===================================================================
Kx  = Y_sol/X_sol 

%===================================================================
disp('*** CHECK#1 = Pos-Def of  X_sol ***')
disp('*** check Pos-Def of  X_sol using CHOLESKY & eigs ***')
[R1,p1] = chol(X_sol);
if p1==0 
    disp('-- CHOLESKY VERIFIES  THAT X_sol is Pos-Def = FEASIBILITY ');
else
    error('-- CHOLESKY VERIFIES  THAT X_sol is NOT Pos-Def');
    %disp('****   CHOLESKY SAYS THAT S is NOT Pos-Def   ***');
end
disp('eigenvalues of X_sol are...');  disp(eig(X_sol)) 


disp('*** CHECK#2 = CLOSED LOOP MATRICES & CL-LOOP EIGENVALUES  ***')
disp(' IMPORTANT!! RECALL & BEWARE THE  PLUS/MINUS SIGN (BE CONSISTENT also in SIMULINK)   SSF_HinfDstab_givenABCD =>  u=+Kx  ')
disp('  CLOSED LOOP MATRICES with a "+" sign since our FuNctions SSF_HinfDstab_givenABCD return =>  u=+Kx    ===  ') 
%========= CLOSED LOOP MATRICES & CL-LOOP EIGENVALUES   =============
Acl0 = Anom + Bnom*Kx ;  % GENERIC with u=+Kx
disp('CL-LOOP EIGS_0= "eig(Acl0 = Anom + Bnom*Kx)"') ; disp (eig(Acl0))
%====   
Acl1 = Avert1 + Bvert1*Kx ;  % GENERIC with u=+Kx
disp('CL-LOOP EIGS_1= "eig(Acl1 = Avert1 + Bvert1*Kx)"') ; disp (eig(Acl1))
%====   
Acl2 = Avert2 + Bvert2*Kx ;  % GENERIC with u=+Kx
disp('CL-LOOP EIGS_2= "eig(Acl2 = Avert2 + Bvert2*Kx)"') ; disp (eig(Acl2))
%====   
Acl3 = Avert3 + Bvert3*Kx ;  % GENERIC with u=+Kx
disp('CL-LOOP EIGS_3 = "eig(Acl3 = Avert3 + Bvert3*Kx)"') ; disp (eig(Acl3))
    
%=========  27Jan20

save('K_quad.mat','Kx')
