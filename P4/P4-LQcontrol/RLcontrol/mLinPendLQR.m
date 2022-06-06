%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inverted pendulum control with LQ control
% Assumes access to the state
%
% J. Miranda Lemos, 2022
% Distributed Estimation and Predictive Control
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%__________________________________________________________________________
% Defines plant parameters
%
g=9.8; % (m/s^2), gravity acceleration
L=0.3; % (m) length of the pendulum
m=0.1; % (kg), mass of the pendulum
k=0.01; % (), friction coefficient

a=g/L;
b=k/m;
c=1/(m*L^2);

umax=100; % maximum value of the control (N.m) 0.04 for 5º

%__________________________________________________________________________
% Linearized model in continuous time

Ap=[0 1; a -b];
Bp=[0; c];
Cp=[1 0]; % assumes as poutput the angle
Dp=0;

%__________________________________________________________________________
% Equivalent discrete-time model with ZOH

% Defines the continuous time model
SYSC=ss(Ap,Bp,Cp,Dp);

% Converts it to discrete time

h=0.1; %(s) sampling period
SYSD=c2d(SYSC,h);

Apd=SYSD.a;
Bpd=SYSD.b;

%__________________________________________________________________________
% LQ controller design (discrete time)clear all

Q=eye(2);
R=1;

[KLQ,SLQ,ELQ] = dlqr(Apd,Bpd,Q,R);

%__________________________________________________________________________
% Simulates the controlled plant

Tmax=10; % (s) Duration of the simulation
sim('LinPendLQ',Tmax);

%__________________________________________________________________________
% Plots results

figure

subplot(211)
gg=plot(t,theta);
set(gg,'LineWidth',1.5);
gg=xlabel('Time (s)');
set(gg,'FontSize',14);
gg=ylabel('\theta (º)');
set(gg,'FontSize',14);

subplot(212)
gg=stairs(u.time,u.signals.values);
set(gg,'LineWidth',1.5);
gg=xlabel('Time (s)');
set(gg,'FontSize',14);
gg=ylabel('u (N.m)');
set(gg,'FontSize',14);

%__________________________________________________________________________
% EoF
