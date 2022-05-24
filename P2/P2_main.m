clear all
close all
clc

A = [1.2 0.8];
R_vector = [0.1 1 10 100 1000 10000];
H_vector = linspace(1,70,70-1+1);

P2_alg(A(1),R_vector,H_vector,1)
P2_alg(A(2),R_vector,H_vector,3)