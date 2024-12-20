clear all;
close all;
clc;
global rho g Ap P Cf L Ip At Ct Q0 d0 d1 d2;

%System Parameters
rho  = 1000; %kg/m^3 water rho 
g = 9.81; %m/s^2 acceleration due to gravity
Ap = 0.1; %m^2 area of pipe from reservoir to turbine
% P = rho * g *dh; %N/m^2 hydrostatic pressure... DIFFERENT dh BETWEEN NODES
Cf = 49000; %kg/m^7 fluid pipe resistance constant
L = 50; %m piper section lengths
Ip = rho *L/Ap; %kg/m^4 fluid inertia of pipe sections
At = 0.75; %m^2 tank areas... FIND MIN TANK AREA BY SIMULATING DIFF VALUES OF At (increments of 0.05 m^2)
Ct = At / (rho *g); %m^5/N fluid capacitance of each tank
Q0 = 1.5; %given 
d0 = 20;
d1= 20; 
d2 = 20; 
h1max = 5 + d0 + d1; %m
h2max = h1max + d2; %m

% Initial conditions
p9_0 = Q0 * Ip; 
p3_0 = Q0 * Ip; 
q7_0 = Ct * ((rho*g*d0) + (rho*g*d1) - (Cf*Q0*abs(Q0))); 
q13_0 = Ct * ((q7_0/Ct) + (rho*g*d2) - (Cf*Q0*abs(Q0))); 
initial = [p9_0, p3_0, q7_0, q13_0]; 

%Time step calcs. there are two natural frequencies, one for each tank, but
%since each tank has the same I and C, the frequencies are the same for
%both.

Tinput = 0.15; %smaller period... 10 data points within the amount of time the input changes.
fninput = 1/Tinput;

omegan = sqrt(1/(Ct*Ip));
fnoutput = omegan/(2*pi);
Toutput = 1/fnoutput; %larger period

maxstepsize = Tinput/10;
starttime = 0;
endtime = 200; %3 x larger period
numsteps = (endtime - starttime)/maxstepsize;

tspan = linspace(starttime,endtime,numsteps);

[t, s] = ode45(@lab4_eqns,tspan,initial);

ext = zeros(length(t),1);
ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab4_eqns(t(i), s(i,:));
end

h1 = s(:,3) / At;
h2 = s(:,4) / At;
H1max = h1max*ones(size(t));
H2max = h2max*ones(size(t));

figure('Name','Heights h','NumberTitle','off','Color','white')
plot(t, h1,'k',t, h2, 'r', t, H1max, 'b', t, H2max, 'g'), grid on
%plot(t, ext(:,1), 'g'), grid on
title('Tank Heights')
ylabel('displacement (m)')
xlabel('time (s)')
legend('height 1', 'height 2', 'max height 1','max height 2')

endtime = 5;
numofsteps = 333;
tspan = linspace(starttime,endtime,numofsteps);
AHHH = zeros(numofsteps);
for i = 1:numofsteps
    AHHH(i) = ext(i);
end

figure('Name','Q0 Input','NumberTitle','off','Color','white')
plot(tspan, AHHH,'b'), grid on
title('Turbine Flow Input')
ylabel('flow rate (m^3/s)')
xlabel('time (s)')