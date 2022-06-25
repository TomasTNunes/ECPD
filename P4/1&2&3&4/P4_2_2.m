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
% Horizon values vector
Hp_vector = 2:1:25;

zblk=1;
ublk=1;

% u Constraints
u_min=-100;
u_max=100;

% du Constraints
du_min=-100;
du_max=100;

% z Constraints
z_min=-100; % rad
z_max=100; % rad

% Vector with R values and Q=1 of Cost Matrix
Q=1; 
R_vector = [0.01,0.1,1,10,100,1000]; 

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

% Simulation characteristics
ref_amp = 0 * pi/180; % rad
ref_step_start = 0; % s
ref_step_time = 11; % s
theta0 = 30 * pi/180; % inicial position(x1) in rad
Tmax=10; % (s) Duration of the simulation

% Loop for R values
for i=1:length(R_vector)
    clear md R inff theta uout rout kt setT overs FG

    R = R_vector(i);
    
    % Loop for Horizon values
    for ii=1:length(Hp_vector)
        clear md inff theta uout rout kt Hp Hu 
        Hp = Hp_vector(ii);
        Hu = Hp;

        % MPC inicialization
        md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);
    
        % Simulates the controlled plant    
        sim('P4_simulink',Tmax);

        % get Step info
        inff = stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.002); % 0.005*|y_f-y_i|, 0.05% chega para ref_amp de 10º mas para outros valores pode ter de variar
        setT(i,ii) = inff.SettlingTime;
        overs(i,ii) = inff.Overshoot/100 * theta0 * 180/pi;
        FG(i,ii) = 10/(2*setT(i,ii) + overs(i,ii));
    end
    
    % Plots Settling time
    figure(1)
    subplot(3,2,i)
    plot(Hp_vector,setT(i,:),'r.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('settling time [s]')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])  % limit due to high overshoots for small predictive Horizons
    end
    
    % Plots Overshoot
    figure(2)
    subplot(3,2,i)
    plot(Hp_vector,overs(i,:),'b.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('overshoot')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])   % limit due to high overshoots for small predictive Horizons
    end
    
    % Plots Merit Figure
    figure(3)
    subplot(3,2,i)
    plot(Hp_vector,FG(i,:),'m.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('FG')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])  % limit due to high overshoots for small predictive Horizons
    end
end
clear md R Hp theta uout rout kt inff

% Best Option based on Settling time, Overshoot and Merit Figure
% R=0.1 & Hp=10

% Horizon
Hp = 10;
Hu=Hp;

% R weight of Cost Matrix
R = 0.1;

% MPC inicialization
md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	         du_max,du_min,u_max,u_min,z_max, ...
	         z_min,Q,R,W,V,h,cmode,solver);
    

% Simulates the controlled plant
Tmax=2; % (s) Duration of the simulation
sim('P4_simulink',Tmax);

% Plots output
figure()
subplot(221)
plot(kt,theta,'r','LineWidth',1.5);
hold on
plot(kt,rout,'--b','LineWidth',1.5)
xlabel('Time (s)');
ylabel('\theta (º)');
title(sprintf('R = %s & Q = %s & H_p = %s & no constraints',string(R),string(Q),string(Hp)))
legend('\theta','\theta_{ref}','Location','NorthEast')

% Plots Control variable
subplot(223)
stairs(uout.time,uout.signals.values,'r','LineWidth',1.5);
xlabel('Time (s)');
ylabel('u (N.m)');


% Simulates the controlled plant
ref_amp = 15 * pi/180; % rad
ref_step_start = 1; % s
ref_step_time = 11; % s
theta0 = 0 * pi/180; % inicial position(x1) in rad
Tmax=4; % (s) Duration of the simulation
sim('P4_simulink',Tmax);

% Plots output
subplot(222)
plot(kt,theta,'r','LineWidth',1.5);
hold on
plot(kt,rout,'--b','LineWidth',1.5)
xlabel('Time (s)');
ylabel('\theta (º)');
title(sprintf('R = %s & Q = %s & H_p = %s & no constraints',string(R),string(Q),string(Hp)))
legend('\theta','\theta_{ref}','Location','SouthEast')

% Plots Control variable
subplot(224)
stairs(uout.time,uout.signals.values,'r','LineWidth',1.5);
xlabel('Time (s)');
ylabel('u (N.m)');