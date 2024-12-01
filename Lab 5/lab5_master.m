%% Part 1 
clear all;
close all;
clc;
global Rw Lw Tm M btau R Gr Cr g Cd rho Af Uin

% Givens
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
tspan=0:0.01:4; %simulating for 4 seconds with 0.01 time step  
[t,s] = ode45(@lab5_eqns,tspan,initial);
pL = s(:,1);
pM = s(:,2);

%ext = zeros(length(t),1);
%ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:)] = lab5_eqns(t(i), s(i,:));
end


figure; 
plot(t, pM/M );
xlabel('Time (s)');
ylabel('Car velocity (m/s)');
title(' Steady state speed ');

%% Part 2
clear all
close all
clc

% Givens
global Rw Lw Tm M btau R Gr Cr g Cd rho Af Uin vref dref uin kp ki
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

kp = 200; %new to part 2 
ki = 300; %new to part 2 

% Define initial conditions
pL0 = 0;
pM0 = 0;
dref0 = 0; %new to part 2 
d0=0; %new to part 2 
initial =[pL0; pM0; dref0; d0]; %new vars to part 2 
tspan =0:0.01:2; % different step for part 2 
[t,s] = ode45(@lab5_eqns,tspan,initial);
pL = s(:,1);
pM = s(:,2);
dref= s(:,3); %new to part 2 
dact= s(:,4); %new to part 2 
for i =1:length(t)
    [ds(i,:), ext(i,:)] = lab5_eqns(t(i),s(i,:)); %different compared to part 1
end
vref= ext(:,1);

figure; 
plot(t, pM/M);
xlabel('Time(s)');
ylabel('Velocity Controller Response');
title( 'Velocity Controller');
hold on
plot (t,vref);

%% Part 3
clear all
close all
clc

% Givens
global Rw Lw Tm M btau R Gr Cr g Cd rho Af Uin vref dref uin kp ki
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

kp = 200;
ki = 300;
%Initial conditions
pL0 = 0;
pM0 = 0;
dref0 = 0; 
d0=0; 
initial =[pL0; pM0; dref0; d0]; 
tspan =0:0.01:300;
[t,s] = ode45(@lab5_eqns,tspan,initial);
pL = s(:,1);
pM = s(:,2);
dref= s(:,3); 
dact= s(:,4);
vref=zeros(length(t),1);

for i=1:length(t)
    vref(i) = LA92Oracle(t(i));
end

for i =1:length(t)
    [ds(i,:), ext(i,:)] = lab5_eqns(t(i),s(i,:)); 
end
vref= ext(:,1);

figure; 
plot(t, pM/M)
xlabel('Time(s)');
ylabel('Velocity');
title( 'Testing');
hold on
plot (t,vref);
