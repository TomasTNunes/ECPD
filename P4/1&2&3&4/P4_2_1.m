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

% Definition of Initial MPC parameters
Hw=1;
Hp=7;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

u_min=-100;
u_max=100;

du_min=-100;
du_max=100;

z_min=-100; % rad
z_max=100; % rad

Q=1; 
R_vector = [0.001,0.1,1,100,1000]; 

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

% Simulation characteristics
ref_amp = 0 * pi/180; % rad
ref_step_start = 0; % s
ref_step_time = 11; % s
theta0 = 30 * pi/180; % inicial position(x1) in rad
Tmax=6; % (s) Duration of the simulation

% plot cloros
p_colors=['b' 'r' 'g' 'c' 'm'];

for i=1:length(R_vector)
    clear md R theta uout rout kt 

    R = R_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);

    % Simulates the controlled plant    
    sim('P4_simulink',Tmax);

    figure(1)
    subplot(211)
    plot(kt,theta,p_colors(i))
    hold on

    subplot(212)
    stairs(uout.time,uout.signals.values,p_colors(i))
    hold on
end

subplot(211)
plot(kt,rout,'k')
xlabel('Time (s)')
ylabel('\theta (º)')
title(sprintf('Q = %s & H_p = %s & no constraints',string(Q),string(Hp)))
legendStrings = "R = " + string(R_vector);
legendStrings(end+1) = "Reference";
legend(legendStrings,'Location','NorthEast');

subplot(212)
xlabel('Time (s)')
ylabel('u (N.m)')

clear legendStrings
R = 1;
Hp_vector = [2,3,15]
Tmax=2; % (s) Duration of the simulation


for i=1:length(Hp_vector)
    clear md Hp theta uout rout kt 

    Hp = Hp_vector(i);
    Hu=Hp;
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);

    % Simulates the controlled plant    
    sim('P4_simulink',Tmax);

    figure(2)
    subplot(211)
    plot(kt,theta,p_colors(i))
    hold on

    subplot(212)
    stairs(uout.time,uout.signals.values,p_colors(i))
    hold on
end

subplot(211)
plot(kt,rout,'k')
xlabel('Time (s)')
ylabel('\theta (º)')
title(sprintf('Q = %s & R = %s & no constraints',string(Q),string(R)))
legendStrings = "H_p = " + string(Hp_vector);
legendStrings(end+1) = "Reference";
legend(legendStrings,'Location','NorthEast');

subplot(212)
xlabel('Time (s)')
ylabel('u (N.m)')
