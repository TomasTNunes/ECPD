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
%Hp=5;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hp_vector = 2:1:25;
%Hu=Hp;

zblk=1;
ublk=1;

u_min=-100;
u_max=100;

du_min=-100;
du_max=100;

z_min=-100; % rad
z_max=100; % rad

Q=1; 
R_vector = [0.001,0.01,0.1,1,10,100,500,1000]; 

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

% Simulation characteristics
ref_amp = 0 * pi/180; % rad
ref_step_start = 0; % s
ref_step_time = 11; % s
theta0 = 10 * pi/180; % inicial position(x1) in rad
Tmax=10; % (s) Duration of the simulation

for i=1:length(R_vector)
    clear md R inf theta uout rout kt setT overs FG

    R = R_vector(i);
    
    for ii=1:length(Hp_vector)
        clear md inf theta uout rout kt Hp Hu 
        Hp = Hp_vector(ii);
        Hu = Hp;

        % MPC inicialization
        md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);
    
        % Simulates the controlled plant    
        sim('P4_simulink',Tmax);
        inf = stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.005); % 0.005*|y_f-y_i|, 0.05% chega para ref_amp de 10º mas para outros valores pode ter de variar
        setT(i,ii) = inf.SettlingTime;
        overs(i,ii) = inf.Overshoot/10;
        FG(i,ii) = 10/(2*setT(i,ii) + overs(i,ii));
    end

    figure(1)
    subplot(4,2,i)
    plot(Hp_vector,setT(i,:),'r.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('settling time [s]')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])  % Hp menores que estes dava overshhots muito altos pq era instavel
    end

    figure(2)
    subplot(4,2,i)
    plot(Hp_vector,overs(i,:),'b.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('overshoot')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])   % Hp menores que estes dava overshhots muito altos pq era instavel
    end

    figure(3)
    subplot(4,2,i)
    plot(Hp_vector,FG(i,:),'g.','LineWidth',1.5)
    title(sprintf('R = %s & Q = %s & no constraints',string(R),string(Q)))
    ylabel('FG')
    xlabel('H_p')
    if i==5 | i==6 | i==7 | i==8
        xlim([i-2 25])  % Hp menores que estes dava overshhots muito altos pq era instavel
    end
end
clear md R Hp theta uout rout kt inf
% 
% Hp = 10;
% Hu=Hp;
% R = 0.1;
% 
% % MPC inicialization
% md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
% 	         du_max,du_min,u_max,u_min,z_max, ...
% 	         z_min,Q,R,W,V,h,cmode,solver);
%     
% % Simulates the controlled plant
% Tmax=4; % (s) Duration of the simulation
% sim('P4_simulink',Tmax);
% 
% figure()
% subplot(211)
% plot(kt,theta,'r','LineWidth',1.5);
% hold on
% plot(kt,rout,'b','LineWidth',1.5)
% xlabel('Time (s)');
% ylabel('\theta (º)');
% title(sprintf('R = %s & Q = %s & H_p = %s & no constraints',string(R),string(Q),string(Hp)))
% legend('\theta','\theta_{ref}','Location','NorthEast')
% 
% subplot(212)
% stairs(uout.time,uout.signals.values,'b','LineWidth',1.5);
% xlabel('Time (s)');
% ylabel('u (N.m)');