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
Hp=10;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

% u Constraints
u_min = -0.22;
u_max = 0.22;

% du Constraints
du_min=-0.085;
du_max=0.085;

% Constraintments of z
z_min=-100;  % rad
z_max=100;   % rad

% Cost Weights
Q=1;
R=0.1; 

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
Tmax=6; % (s) Duration of the simulation

% Simulates the controlled plant without disturbance 
sim('P4_simulink',Tmax);

% Plot outputs and control variables
figure()
subplot(321)
plot(kt,theta,'b','LineWidth',1.5)
xlabel('Time (s)');
ylabel('\theta (º)');
hold on
subplot(323)
stairs(uout.time,uout.signals.values,'b','LineWidth',1.5);
xlabel('Time (s)');
ylabel('u (N.m)');
hold on

clear theta kt uout rout
% Disturbance
T = 0.8; % width of pulse in seconds 
FB = 2/T; % fractional bandwidth
T_mp1 = 1; % time of the max pulse 1 in seconds
t1=linspace(-T,T,100);
[~,~,ye1] = gauspuls(t1,1,FB); % disturbance pulse 1
t1=t1+T_mp1;  % translate pulse 1
T_mp2 = 3; % time of the max pulse 2 in seconds
t2=linspace(-T,T,100);
[~,~,ye2] = gauspuls(t2,1,FB); % disturbance pulse 2
t2=t2+T_mp2;  % translate pulse 2
t= [0 t1 t2 Tmax]; % total disturbance time
ye= [0 ye1 ye2 0]; % total disturbance values
ye = 0.05*ye; % disturbance amplitude
pulse_dist = [t' ye']; % simulink input

% Simulates the controlled plant w/ disturbance    
sim('P4_disturbance_sim',Tmax);

% Plots Disturbance
subplot(3,2,5:6)
plot(t,ye,'b','LineWidth',1.5);
xlabel('Time (s)');
ylabel('u (N.m)');
title('Disturbance in the control variable')

% Plots outputs and control variables
subplot(321)
plot(kt,theta,'r','LineWidth',1.5);
hold on
plot(kt,rout,'--k','LineWidth',1.5)
title_str = 'Q = '+string(Q) + ' & R = '+string(R) + ' & H = '+string(Hp) +  ' & |u| < '+string(u_max) + ' & |\Delta u| < '+string(du_max);
title(title_str)
legend('\theta_{w/o dist}','\theta_{w/ dist}','reference','Location','NorthEast')
subplot(323)
stairs(uout.time,uout.signals.values,'r','LineWidth',1.5);

clear theta kt uout rout uout_MPC
% Simulation characteristics
ref_amp = 15 * pi/180; % rad
ref_step_start = 0.5; % s
ref_step_time = 11; % s
theta0 = 0 * pi/180; % inicial position(x1) in rad
Tmax=6; % (s) Duration of the simulation

% Simulates the controlled plant without disturbance 
sim('P4_simulink',Tmax);

% Plots outputs and control variables
subplot(322)
plot(kt,theta,'b','LineWidth',1.5)
hold on
xlabel('Time (s)');
ylabel('\theta (º)');
subplot(324)
stairs(uout.time,uout.signals.values,'b','LineWidth',1.5);
hold on
xlabel('Time (s)');
ylabel('u (N.m)');

clear theta kt uout rout
% Simulates the controlled plant w/ disturbance    
sim('P4_disturbance_sim',Tmax);

% Plots outputs and control variables
subplot(322)
plot(kt,theta,'r','LineWidth',1.5);
hold on
plot(kt,rout,'--k','LineWidth',1.5)
title_str = 'Q = '+string(Q) + ' & R = '+string(R) + ' & H = '+string(Hp) +  ' & |u| < '+string(u_max) + ' & |\Delta u| < '+string(du_max);
title(title_str)
legend('\theta_{w/o dist}','\theta_{w/ dist}','reference','Location','SouthEast')
subplot(324)
stairs(uout.time,uout.signals.values,'r','LineWidth',1.5);
