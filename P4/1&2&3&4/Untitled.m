clear all
close all
clc
t= -5:0.1:5;
[~,~,ye] = gauspuls(t,1,2); 
figure()
plot(t,ye)
