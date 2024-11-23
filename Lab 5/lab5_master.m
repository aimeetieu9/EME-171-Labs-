clear all;
close all;
clc;
global Rw Lw Tm M btau R Gr Cr g Cd rho Af Uin
Rw = 0.3; % ohms
Lw = 0.015; % henry
Tm = 1.718; % Weber
M = 2200; % kg
btau = 0.05; % Nms/rad
R = 0.2; % m
Gr = 5; % ratio
Cr = 0.006; % rolling resistance coefficient
g = 9.81; % m/s^2
Cd = 0.32; % drag coefficient
rho = 1.21; % kg/m^3
Af = 2.05; % m^2
Uin = 100; %volts

% Define initial conditions
pL0 = 0;
pM0 = 0;
initial =[pL0,pM0];
tspan=0:0.1:1; %simulating for 4 seconds with 0.01 time step  
[t,s] = ode45(@lab5_eqns,tspan,initial);
pL = s(:,1);
pM = s(:,2);

%ext = zeros(length(t),1);
%ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab4_eqns(t(i), s(i,:));
end


figure; 
plot(t, pM/M );
xlabel('Time (s)');
ylabel('Car velocity (m/s)');
title(' Steady state speed ');
