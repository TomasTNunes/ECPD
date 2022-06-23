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
Hp=10;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

% Vectors with Constraintments of u and delta u
u_vector = [0.22 0.15];
du_vector = [0.085 0.15];

% Constraintments of z
z_min=-100;  % rad
z_max=100;   % rad

% Vector with R values and Q=1 of Cost Matrix
Q=1;
R_vector=[0.1 100]; 

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

% Loop for R values
for i=1:length(R_vector)

    R = R_vector(i);
    
    % Loop for constraintments values
    for ii=1:length(u_vector)
        clear u_max u_min du_max du_min inff theta theta_LQR kt t u_LQR md uout rout

        u_max = u_vector(ii);
        u_min = -u_vector(ii);
        du_max = du_vector(ii);
        du_min = -du_vector(ii);

        % MPC inicialization
        md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);

        % LQR
        KLQ = dlqr(Apd,Bpd,eye(2),R);

        % Simulates the controlled plant with MPC   
        sim('P4_simulink',Tmax);

        % get Step info
        inff=stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.002);
        setT(i,ii) = inff.SettlingTime; % settling time
        overs(i,ii) = inff.Overshoot/100 * theta0 * 180/pi; % overshoot
        FG(i,ii) = 10/(2*setT(i,ii) + overs(i,ii)); % Merit Figure

        %setT(i,ii) = inff.SettlingTime; % settling time
        %overs(i,ii) = inff.Overshoot/100 * theta0 * 180/pi; % overshoot
        %FG(i,ii) = 10/(2*setT(i,ii) + overs(i,ii)); % Merit Figure

        % Simulates the controlled plant with LQR   
        sim('P4_sim_LQR',Tmax);

        % Plots the outputs and reference
        figure(2*i+ii-2)
        subplot(211)
        plot(kt,theta,'r','Linewidth',1.5)
        hold on
        plot(t,theta_LQR,'b','Linewidth',1.5)
        plot(kt,rout,'k','Linewidth',1.5)
        xlabel('time (s)');
        ylabel('\theta (º)');
        legend('\theta_{MPC}','\theta_{LQR}','\theta_{ref}','Location','NorthEast')
        title_str = 'Q = '+string(Q) + ' & R = '+string(R) + ' & H = '+string(Hp) + ' & |u| < '+string(u_max) + ' & |\Delta u| < '+string(du_max);
        title(title_str)

        % Plots the control variable
        subplot(212)
        stairs(uout.time,uout.signals.values,'r','Linewidth',1.5);
        hold on
        stairs(u_LQR.time,u_LQR.signals.values,'b','Linewidth',1.5);
        xlabel('time (s)');
        ylabel('u (N.m)');

    end
end 
clear u_max u_min du_max du_min inff theta theta_LQR kt t u_LQR md uout rout

% Effect of enlarging the Horizon 
% It was only done for a big R (R=100), since for small R the effect is unnoticeable

% R weight
R = 100

% u Constraints
u_max = 0.22;
u_min = -0.22;

% du Constraints
du_max = 0.085;
du_min = -0.085;

% Horizon vector
Hp_vector = [4 5 25];

% plot colors
p_colors=['b' 'r' 'g' 'c' 'm'];

Tmax=5; % (s) Duration of the simulation

% Loop for Horizon values
for i=1:length(Hp_vector)
    clear kt theta uout rout md Hp Hu inff
    Hp = Hp_vector(i);
    Hu=Hp;
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
	            du_max,du_min,u_max,u_min,z_max, ...
	            z_min,Q,R,W,V,h,cmode,solver);

    % Simulates the controlled plant with MPC   
    sim('P4_simulink',Tmax);

    % get Step info
    inff=stepinfo(-theta,kt,-ref_amp*180/pi,-theta0*180/pi,'SettlingTimeThreshold',0.002);
    setT2(i) = inff.SettlingTime; % settling time
    overs2(i) = inff.Overshoot/100 * theta0 * 180/pi; % overshoot
    FG2(i) = 10/(2*setT2(i) + overs2(i)); % Merit Figure

    % Plots the outputs
    figure(5)
    subplot(211)
    plot(kt,theta,p_colors(i),'Linewidth',1.5)
    hold on
    xlabel('time (s)');
    ylabel('\theta (º)'); 
    title_str = 'Q = '+string(Q) + ' & R = '+string(R) + ' & |u| < '+string(u_max) + ' & |\Delta u| < '+string(du_max);
    title(title_str)

    % Plots the control variable
    subplot(212)
    stairs(uout.time,uout.signals.values,p_colors(i),'Linewidth',1.5);
    hold on
    xlabel('time (s)');
    ylabel('u (N.m)');

end
% Plots reference
subplot(211)
plot(kt,rout,'k','Linewidth',1.5)
legendStrings = "H_p = " + string(Hp_vector);
legendStrings(end+1) = "Reference";
legend(legendStrings,'Location','NorthEast');
