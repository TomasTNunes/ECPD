clear all
close all
clc

A = [1.2 0.8];  % vector with A matrix for both plants
R_vector = [0.1 1 10 100 1000 10000];  % values of cost R with Q=1
H_vector = linspace(1,70,70-1+1);  % values of Horizon

% Run algorithm for 1st plant
P2_alg(A(1),R_vector,H_vector,1)
% Run algorithm for 2nd plant
P2_alg(A(2),R_vector,H_vector,3)