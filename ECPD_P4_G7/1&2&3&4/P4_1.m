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
% Horizon values
Hw=1;
Hp=3;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

% u Constraints
u_min = -100;
u_max = 100;

% du Constraints
du_min=-100;
du_max=100;

% Constraintments of z
z_min=-100;  % rad
z_max=100;   % rad

% Cost Weights
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
ref_amp = 0 * pi/180; % rad
ref_step_start = 0; % s
ref_step_time = 11; % s
theta0 = 30 * pi/180; % inicial position(x1) in rad
Tmax=3; % (s) Duration of the simulation

% Simulates the controlled plant    
sim('P4_simulink',Tmax);

% Plots output
figure()
subplot(211)
plot(kt,theta,'r','LineWidth',1.5);
hold on
plot(kt,rout,'--b','LineWidth',1.5)
xlabel('Time (s)');
ylabel('\theta (º)');
title(sprintf('R = %s & Q = %s & H_p = %s & no constraints',string(R),string(Q),string(Hp)))
legend('\theta','\theta_{ref}','Location','NorthEast')

% Plots contol variable
subplot(212)
stairs(uout.time,uout.signals.values,'r','LineWidth',1.5);
xlabel('Time (s)');
ylabel('u (N.m)');