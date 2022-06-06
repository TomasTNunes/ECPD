clear all
close all
clc
path(path,'../')

h=0.1; % sampling period (s)

% Defines plant parameters
g=9.8; % (m/s^2), gravity acceleration
L=0.3; % (m) length of the pendulum
m=0.1; % (kg), mass of the pendulum
k=0.01; % (), friction coefficient

a=g/L;
b=k/m;
c=1/(m*L^2);

% Linearized model in continuous time
Ap=[0 1; a -b];
Bp=[0; c];
Cp=eye(2); 
Dp=[0; 0];

% Defines the continuous time model
SYSC=ss(Ap,Bp,Cp,Dp);

% Converts it to discrete time
SYSD=c2d(SYSC,h);
Apd=SYSD.a;
Bpd=SYSD.b;

% Model used by MPC
Ad=Apd;
Bd=Bpd;
Cyd=Cp;

Czd=Cp(1,:);
Dzd=Dp(1);

Ccd=Cp(1,:);
Dcd=Dp(1);

% Definition of MPC parameters
Hw=1;
Hp=5;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

u_min=-100;
u_max=100;

du_min=-100;
du_max=100;

z_min=-100;
z_max=100;

Q=1; 
R=1; 

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

% MPC inicialization
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	    du_max,du_min,u_max,u_min,z_max, ...
	    z_min,Q,R,W,V,h,cmode,solver);

% Simulation characteristics
ref_amp = 0; % rad
ref_freq = 1/10; % Hz
theta0 = 20 * pi/180; % inicial position(x1) in rad
Tmax=10; % (s) Duration of the simulation


% Simulates the controlled plant
sim('P4_simulink',Tmax);

% Plots results
figure()
subplot(211)
gg=plot(kt,theta,'b');
set(gg,'LineWidth',1.5);
hold on
plot(kt,rout,'r')
gg=xlabel('Time (s)');
set(gg,'FontSize',14);
gg=ylabel('\theta (º)');
set(gg,'FontSize',14);

subplot(212)
gg=stairs(uout.time,uout.signals.values);
set(gg,'LineWidth',1.5);
gg=xlabel('Time (s)');
set(gg,'FontSize',14);
gg=ylabel('u (N.m)');
set(gg,'FontSize',14);

