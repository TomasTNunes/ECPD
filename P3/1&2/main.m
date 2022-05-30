clear all
clc
close all

path(path,'../')

tfinal=9; % duration of the simulation (s)

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
Hp=3;   % Minimum Hp=2; corresponds to 1-step prediction horizon
Hu=Hp;

zblk=1;
ublk=1;

% Default values

u_min=-100;
u_max=100;

du_min=-100;
du_max=100;

z_min=-10000;
z_max=10000;

Q=1000;   %1*eye(2);
R=10000; %0.00001;

W=[]; % state estimator not used
V=[]; % 

cmode=0; % feedback of the state
solver='qp_as';

u_min_vector=-[1,0.4,0.3];
u_max_vector=[1,0.4,0.3];

du_min_vector=-[1,0.2,0.15,0.13];
du_max_vector=[1,0.2,0.15,0.13];

z_min_vector=-[2,1.15,1,0.5];
z_max_vector=[2,1.15,1,0.5];

Q_vector=[1,1,1,1,1];
R_vector=[1,0.1,0.001,10,100];

Hp_vector=[2,3,50];


for i=1:length(u_min_vector)
    clear md u_min u_max kt yout uout
    u_min = u_min_vector(i);
    u_max = u_max_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(1)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end
% instavel apartir umax umin +/-0.22

u_min=-100;
u_max=100;

for i=1:length(du_min_vector)
    clear md du_min du_max kt yout uout
    du_min = du_min_vector(i);
    du_max = du_max_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(2)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end
% instavel apartir dumax dumin +/-0.1

du_min=-100;
du_max=100;

for i=1:length(z_min_vector)
    clear md z_min z_max kt yout uout
    z_min = z_min_vector(i);
    z_max = z_max_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(3)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end

z_min=-10000;
z_max=10000;

for i=1:length(Q_vector)
    clear md Q R kt yout uout
    Q = Q_vector(i);
    R = R_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(4)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end
% instavel apartir Q=1 R=150 => R/Q > 150 instavel

Q=1000;
R=10000;

for i=1:length(Hp_vector)
    clear md Hp kt yout uout
    Hp = Hp_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(5)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end
clear Hp_vector

u_min=-0.45;
u_max=0.45;

du_min=-0.13;
du_max=0.13;

z_min=-1.5;
z_max=2;

Q=1;
R=100;

Hp_vector=[2,3,5,15,90];

for i=1:length(Hp_vector)
    clear md Hp kt yout uout
    Hp = Hp_vector(i);
    
    % MPC inicialization
    md = MPCInit(Ad,Bd,Cyd,Czd,Dzd,Ccd,Dcd,Hp,Hw,zblk,Hu,ublk, ...
            du_max,du_min,u_max,u_min,z_max, ...
            z_min,Q,R,W,V,h,cmode,solver);
        
    sim('Prob1_2_simulink');
    
    % Plots the output and the reference
    figure(6)
    subplot(2,1,1)
    
    plot(kt,yout);
    hold on
    plot(kt,rout,'r');
    xlabel('time (s)');
    ylabel('y');

    % Plots the control variable
    subplot(2,1,2)

    stairs(kt,uout);
    hold on
    xlabel('time (s)');
    ylabel('u');
end
% aparitr de H=100 existe problemas nas funçoes do MPC para esta
% configuração

    