clear all
close all
clc

% -----------------------------------------
A = 1.2;
B = 1;

Q = 1;
R = 1;

[KLQ,S,lambda] = dlqr(A,B,Q,R);

% -----------------------------------------

C = 1;

H = 5;
W = zeros(H,H);

for i=1:H
    for ii=1:i
        W(i,ii) = C*A^(i-ii)*B;
    end

    PI(i,1) = C*A^i;
end

M = W'*W + R*eye(H);

e1 = zeros(1,H);
e1(1) = 1;

KRH = e1*M^(-1)*W'*PI;

eignRH = eig(A-B*KRH);

% -----------------------------------------
H_vector = linspace(1,200,200-1+1);

for j=1:length(H_vector)
    clear H W PI M e1
    H = H_vector(j);
    W = zeros(H,H);

    for i=1:H
        for ii=1:i
            W(i,ii) = C*A^(i-ii)*B;
        end
        PI(i,1) = C*A^i;
    end
    M = W'*W + R*eye(H);
    e1 = zeros(1,H);
    e1(1) = 1;
    KRH_vector(j) = e1*M^(-1)*W'*PI;
    eignRH_vector(j) = eig(A-B*KRH_vector(j));
end

figure(1)
plot(H_vector,KRH_vector)
hold on
plot(H_vector,zeros(1,length(H_vector))+KLQ)

figure(2)
th=linspace(0,2*pi,100);
xc = cos(th);
yc = sin(th);
bound_line = linspace(0,1,100);
%plot(xc,yc)
plot(bound_line,zeros(1,length(bound_line)))
hold on
plot(eignRH_vector,zeros(1,length(eignRH_vector)),'.')


