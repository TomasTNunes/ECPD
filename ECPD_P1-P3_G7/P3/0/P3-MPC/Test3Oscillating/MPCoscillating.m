% Simulation of an oscillating 2nd order linear process controlled by
% an MPC controller using Simulink.
%
% Uses MPCtools 1.0 package, J. Akesson, Univ. Lund, 2006.
%
% Plant to control:
% ZOH of the continuous 2nd order plant
%
% Hc(s)=w_n^2/(s^2+2.csi.w_n.s+s^2)
% with the zeros removed.
%
% Distributed Predictive Control and Estimation - MEEC
% J. Miranda Lemos 2022
%__________________________________________________________________________

clear all
path(path,'../')

tfinal=150; % duration of the simulation (s)

%__________________________________________________________________________
% Parameters of the continuous 2nd order linear process

wn=1;
csi=0.1;

% Parameters of the discrete model

h=0.1; % sampling period (s)

w=wn*sqrt(1-csi^2);
alpha=exp(-csi*wn*h);
beta=cos(w*h);
gamma=sin(w*h);

% b1=1-alpha*(beta+csi*wn*gamma/w);
% b2=alpha^2+alpha*(+csi*wn*gamma/w-beta);

a1=-2*alpha*beta;
a2=alpha^2;

Ap=[0 1; -a2 -a1];
Bp=[0; 1];
%Cp=[b1 b2];
Cp=eye(2);   %Assumes access to the state as output
Dp=[0; 0];

% Model used by MPC

Ad=Ap;
Bd=Bp;
Cyd=Cp;

Czd=[1 0];
Dzd=0;   % Dp;

Ccd=Cp;
Dcd=Dp;

% Definition of MPC parameters

Hw=1;
Hp=3;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

u_min=-100;
u_max=100;

du_min=-100;
du_max=100;

z_min=-10000*[1;1];
z_max=10000*[1;1];

Q=1000;   %1*eye(2);
R=10000; %0.00001;

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

sim('Oscillating');

%--------------------------------------------------------------------------
% Plots results

figure(1)

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

