clear all;
close all;
clc;
global density g Ap P Cf L Ip At Ct;

%System Parameters
density = 1000; %kg/m^3 water density
g = 9.81; %m/s^2 acceleration due to gravity
Ap = 0.1; %m^2 area of pipe from reservoir to turbine
P = density*g*dh; %N/m^2 hydrostatic pressure... DIFFERENT dh BETWEEN NODES
Cf = 49000; %kg/m^7 fluid pipe resistance constant
L = 50; %m piper section lengths
Ip = density*L*Ap; %kg/m^4 fluid inertia of pipe sections
At = ; %m^2 tank areas... FIND MAX TANK AREA BY SIMULATING DIFF VALUES OF At (increments of 0.05 m^2)
Ct = At / (density*g); %m^5/N fluid capacitance of each tank