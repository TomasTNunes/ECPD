clear all
clc
%close all
path(path,'../')

tfinal=10; % duration of the simulation (s)

%__________________________________________________________________________
% Parameters of the continuous 2nd order linear process

% Parameters of the discrete model

h=0.1; % sampling period (s)

Ap=1.2;
Bp=1;
Cp=1;   %Assumes access to the state as output
Dp=0;

% Model used by MPC

Ad=Ap;
Bd=Bp;
Cyd=Cp;

Czd=Cp;
Dzd=Dp; 

Ccd=Cp;
Dcd=Dp;

% Definition of MPC parameters

Hw=1;
Hp=90;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

u_min=-0.45;
u_max=0.45;

du_min=-0.13;
du_max=0.13;

z_min=-1.5;
z_max=2;

Q=1;   %1*eye(2);
R=100; %0.00001;

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

%--------------------------------------------------------------------------
% MPC inicialization

md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,cmode,solver);

%--------------------------------------------------------------------------
% Simulates the controlled system

sim('Prob1_2_simulink');

%--------------------------------------------------------------------------
% Plots results

figure()

% Plots the output and the reference
subplot(2,1,1)

gg=plot(kt,yout,'b');
set(gg,'LineWidth',1.5);
hold on
gg=plot(kt,rout,'r');
set(gg,'LineWidth',1.5);
hold off

gg=xlabel('time (s)');
set(gg,'Fontsize',14);
gg=ylabel('y');
set(gg,'Fontsize',14);

% Plots the control variable
subplot(2,1,2)

gg=stairs(kt,uout);
set(gg,'LineWidth',1.5);
gg=xlabel('time (s)');
set(gg,'Fontsize',14);
gg=ylabel('u');
set(gg,'Fontsize',14);

%__________________________________________________________________________
% End of file

