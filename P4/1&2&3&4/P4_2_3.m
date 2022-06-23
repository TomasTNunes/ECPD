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
Hp=10;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;


u_vector = [0.4 0.25 0.15];
du_vector = [0.3 0.15 0.1 0.085];

z_min=-100; % rad
z_max=100; % rad

Q=1; 
R = 0.1; 

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

% Simulation characteristics
ref_amp = 0 * pi/180; % rad
ref_step_start = 0; % s
ref_step_time = 11; % s
theta0 = 30 * pi/180; % inicial position(x1) in rad
Tmax=3; % (s) Duration of the simulation

% plot colors
p_colors=['b' 'r' 'g' 'c' 'm'];

du_min = -100;
du_max = 100;
for i=1:length(u_vector)
    clear u_min u_max theta kt uout inff md

    u_min = -u_vector(i);
    u_max = u_vector(i);


    % MPC initialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	        du_max,du_min,u_max,u_min,z_max, ...
	        z_min,Q,R,W,V,h,cmode,solver);

    % Simulates the controlled plant    
    sim('P4_simulink',Tmax);
    inff = stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.002); % 0.005*|y_f-y_i|, 0.05% chega para ref_amp de 10º mas para outros valores pode ter de variar
    setT(1,i) = inff.SettlingTime;
    overs(1,i) = inff.Overshoot/100 * theta0 * 180/pi;
    FG(1,i) = 10/(2*setT(1,i) + overs(1,i));

    % plot u
    figure(1)
    subplot(2,1,1)
    plot(kt,theta,p_colors(i),'Linewidth',1.5);
    hold on
    xlabel('time (s)');
    ylabel('\theta (º)');

    subplot(2,1,2)
    stairs(uout.time,uout.signals.values,p_colors(i),'Linewidth',1.5)
    hold on
    xlabel('time (s)');
    ylabel('u (N.m)');
end
%
subplot(211)
plot(kt,rout,'k','Linewidth',1.5)
legendStrings = "|u| < " + string(u_vector);
legendStrings(end+1) = "Reference";
legend(legendStrings,'Location','NorthEast');


u_min = -100;
u_max = 100;
for i=1:length(du_vector)
    clear du_min du_max theta kt uout inff md

    du_min = -du_vector(i);
    du_max = du_vector(i);

    % MPC initialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	        du_max,du_min,u_max,u_min,z_max, ...
	        z_min,Q,R,W,V,h,cmode,solver);

    % Simulates the controlled plant    
    sim('P4_simulink',Tmax);
    inff = stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.002); % 0.005*|y_f-y_i|, 0.05% chega para ref_amp de 10º mas para outros valores pode ter de variar
    setT(2,i) = inff.SettlingTime;
    overs(2,i) = inff.Overshoot/100 * theta0 * 180/pi;
    FG(2,i) = 10/(2*setT(2,i) + overs(2,i));

    figure(2)
    subplot(2,1,1)
    plot(kt,theta,p_colors(i),'Linewidth',1.5);
    hold on
    xlabel('time (s)');
    ylabel('\theta (º)');

    subplot(2,1,2)
    stairs(uout.time,uout.signals.values,p_colors(i),'Linewidth',1.5)
    hold on
    xlabel('time (s)');
    ylabel('u (N.m)');
end
%
subplot(211)
plot(kt,rout,'k','Linewidth',1.5)
legendStrings = "|\Delta u| < " + string(du_vector);
legendStrings(end+1) = "Reference";
legend(legendStrings,'Location','NorthEast');
