% tic

clear all
close all
clc

%% Plots function level curves
% Range of independent variables to consider 
x1min=-3;
x1max=3;
x2min=-3;
x2max=3;

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
        ff(ii,jj)=Prob2Function(x);
    end
end

% Plots the level curves using the Matlab function contourf
Nlevel=25;  % Number of level curves in the contour plot
LW = 'linewidth'; FS = 'fontsize'; MS = 'markersize';
figure(1), contourf(xv1,xv2,ff,Nlevel,LW,1.2), colorbar, colormap default
axis([x1min x1max x2min x2max]), axis square
hold on

%% Compute and Plots important points
% Various initial guesses that result in different unconstrained minima
x0_matrix = [1 -1; -1 1; 1.5 1.5; -0.3 -0.2; 0 -2; -2 0; 2 0; 0 0.5; 0 0]';

for i=1:length(x0_matrix)
    x0 = x0_matrix(:,i);

    % options = optimoptions('fminunc','Algorithm','quasi-newton','Display','off');
    options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true,'Display','off');   % marginally improves performance

    % Uses the solver fminunc to compute the minimum of the function defined in
    % the Matlab function defined in the file Prob2Function.m
    xopt=fminunc(@Prob2Function,x0,options)

    % Hessian matrix is [12*x(1)^2-20 0; 0 12*x(2)^2-20]
    Hessian = [12*xopt(1)^2-20 0; 0 12*xopt(2)^2-20];
    % Compute eigenvalues to perform the second-derivative test
    lambda = eig(Hessian); 

    % Plots the initial point as a circle
    if (lambda(1) > 0 && lambda(2) > 0)         % red for minimum
        gg=plot(x0(1),x0(2),'or');
    elseif (lambda(1) < 0 && lambda(2) < 0)     % black for maximum
        gg=plot(x0(1),x0(2),'ok');
    elseif lambda(1)*lambda(2) < 0              % blue for saddle point
        gg=plot(x0(1),x0(2),'ob');
    else                                        % white if second-derivative test is inconclusive
        gg=plot(x0(1),x0(2),'ow');
    end
    set(gg,'Linewidth',1.5);

    % Plots the final estimate of the unconstrained minimum as a cross
    if (lambda(1) > 0 && lambda(2) > 0)         % red for minimum
        gg=plot(xopt(1),xopt(2),'xr');
    elseif (lambda(1) < 0 && lambda(2) < 0)     % black for maximum
        gg=plot(xopt(1),xopt(2),'xk');
    elseif lambda(1)*lambda(2) < 0              % blue for saddle point
        gg=plot(xopt(1),xopt(2),'xb');
    else                                        % white if second-derivative test is inconclusive
        gg=plot(xopt(1),xopt(2),'xw');
    end
    set(gg,'Linewidth',1.5);

    % Identifies axis
    gg=xlabel('x_1');
    set(gg,'FontSize',14);

    gg=ylabel('x_2');
    set(gg,'FontSize',14);

end

hold off

%% Plots the 3d view of the function

figure(2)
surf(xx1,xx2,ff);

% Identifies axis
gg=xlabel('x_1');
set(gg,'FontSize',14);

gg=ylabel('x_2');
set(gg,'FontSize',14);

gg=zlabel('f(x)');
set(gg,'FontSize',14);

%% Definines Boundary
% initial estimate of the desired minimum
x0 = [2;2];
% reference minimum
options = optimoptions('fminunc','Algorithm','quasi-newton','Display','off');  % without gradient
xref = fminunc(@Prob2Function,x0,options)

% Auxiliar variables
% angles of lines
dalpha = 1;
alpha = 0:dalpha:180-dalpha;
alpha(alpha==90) = 89.9;
% vector with steps for each iteration
step = [20 6 1.5 0.2];
length_step = length(step);
% vector that records all the ponits that converge to minimum
vec = xref';
% vector with boundary points of lines with angle from 0º to 180º
vecext1 = [zeros(length(alpha),1),zeros(length(alpha),1)];
% vector with boundary points of lines with angle from 180º to 360º
vecext2 = [zeros(length(alpha),1),zeros(length(alpha),1)];
% limit to converge
conv_limit = 0.1;
i = 2;
ii1 = 1;
ii2 = 1;

for j=1:length(alpha)
    m = tan(alpha(j)*pi/180);    % slope of line
    b = xref(2) - m*xref(1);     % y(x=0)
    y = @(x) m*x + b;            % line function
    run = true;                  % run/stop condition dor while loops
    xaux = xref;
    jj = 1;
    % Computes for angles from 0º to 180º
    while(run)
        xaux(1) = xaux(1) + step(jj)*cos(alpha(j)*pi/180);
        xaux(2) = y(xaux(1));
        xopt = fminunc(@Prob2Function,xaux,options);

        % saves point if converged
        if abs(xopt-xref) <= conv_limit
            vec(i,:) = xaux';
            i = i+1;
        % decrease step if didn't converge
        elseif jj ~= length_step
            xaux(1) = xaux(1) - step(jj)*cos(alpha(j)*pi/180);
            jj = jj+1;
        % saves boundary point and stops while loop
        else
            vecext1(ii1,:) = vec(i-1,:);
            ii1 = ii1+1;
            run = false;
        end 
    end 
    run = true;
    xaux = xref;
    jj = 1;
     % Computes for angles from 180º to 360º
    while(run)
        xaux(1) = xaux(1) - step(jj)*cos(alpha(j)*pi/180);
        xaux(2) = y(xaux(1));
        xopt = fminunc(@Prob2Function,xaux,options);

        % saves point if converged
        if abs(xopt-xref) <= conv_limit
            vec(i,:) = xaux';
            i = i+1;
        % decrease step if didn't converge
        elseif jj ~= length_step
            xaux(1) = xaux(1) + step(jj)*cos(alpha(j)*pi/180);
            jj = jj+1;
        % saves boundary point and stops while loop
        else
            vecext2(ii2,:) = vec(i-1,:);
            ii2 = ii2+1;
            run = false;
        end 
    end 
end

% Boundary vector
vecext = [vecext1; vecext2];
vecext(end+1,:) = vecext(1,:);

% Plots boundary and points that converged
figure(3)
plot(vec(:,1),vec(:,2),'b.')
hold on
plot(vecext(:,1),vecext(:,2),'r-','Linewidth',1.5)

% Identifies axis
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);

% Legend and Title
legend('converged points','boundary','Location','Northwest');
title(sprintf('Boundary of the minimum (%.3f,%.3f) atraction basin',xref));

% toc