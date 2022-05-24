% Finds the minimum of a quadratic function.
%
% Matlab sw required: optimization toolbox
% Functions called
%    fminunc - Matlab function (optimization toolbox) for uncionstrained
%        minimization
%    BasicFunction.m - user defined function that defines the function to be
%        minimized
%
% IST, Distributed Predictive Control and Estimation
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Plots function level curves

% Range of independent variables to consider 
x1min=-2;
x1max=2;
x2min=-2;
x2max=2;

% Number of intervals in the mesh grid
N1=100;
N2=100;

xv1 = linspace(x1min,x1max,N1);
xv2 = linspace(x2min,x2max,N2);
[xx1,xx2] = meshgrid(xv1,xv2);

% Computes the function at the different points of the mesh grid
for ii=1:N1
    for jj=1:N2
        x=[xx1(ii,jj); xx2(ii,jj)];
        ff(ii,jj)=p1(x);
    end
end

% Plots the level curves using the Matlab function contoutr
Nlevel=40;  % Number of level curves in the contour plot
LW = 'linewidth'; FS = 'fontsize'; MS = 'markersize';
figure(1), contour(xv1,xv2,ff,Nlevel,LW,1.2), colorbar
axis([x1min x1max x2min x2max]), axis square
title('Level Curves')
hold on

%--------------------------------------------------------------------------
% Compute the minimum

% Initial estimate of the minimum
x0=[-1; 1];

% Define the options to be used with the fminunc solver:
% The quasi-newton algorithm is used because it does not require the
% gradient; the default algorithm requires the gradient
options = optimoptions('fminunc','Algorithm','quasi-newton');

% Uses the solver fminunc to compute the minimum of the function defined in
% the Matlab function defined in the file BasicFunction.m
xopt=fminunc(@p1,x0,options);

%--------------------------------------------------------------------------
% Computes the constrained minimum associated to the constraint 
% x(1) <= 0.5

Ac=[1 0];
Bc=0.5;
xoptconstr=fmincon(@p1,x0,Ac,Bc);

%--------------------------------------------------------------------------
% Plots the initial point as a red circle
gg=plot(x0(1),x0(2),'or');
set(gg,'Linewidth',1.5);

% Plots the final estimate of the unconstrained minimum as a red cross
gg=plot(xopt(1),xopt(2),'xr');
set(gg,'Linewidth',1.5);

% Plots the final estimate of the constrained minimum as a red star
gg=plot(xoptconstr(1),xoptconstr(2),'*r');
set(gg,'Linewidth',1.5);

%plots the constraint boundary

z2c=[x2max:-0.1:x2min];
z1c=0.5*ones(1,length(z2c));
gg=plot(z1c,z2c,'k');
set(gg,'Linewidth',1.5);

% Identifies axis
gg=xlabel('x_1');
set(gg,'FontSize',14);

gg=ylabel('x_2');
set(gg,'FontSize',14);

hold off

%--------------------------------------------------------------------------
% Plots the 3d view of the function

figure(2)
surf(xx1,xx2,ff);

% Identifies axis
gg=xlabel('x_1');
set(gg,'FontSize',14);

gg=ylabel('x_2');
set(gg,'FontSize',14);

gg=zlabel('f(x)');
set(gg,'FontSize',14);

title('3D View of the Function');
%--------------------------------------------------------------------------
% End of File