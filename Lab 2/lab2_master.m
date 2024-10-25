clear all;
close all;
clc;
global M K B amp Vc L A Mu Kt fn omega zeta q6_0 g;

%parameters
amp = 0.1; %m/s, velocity input (road profile)
g = 9.81; %m/s^2;

%system parameters
Vc = 10; %m/s
L = 1.2; %m
A = 0.08; %m
M = 250; %kg, mass of 1/4 car
Mu = M / 5; %M/Mu = 5
fn = 1; % = 1 Hz
omega = fn*2*pi;
K = omega^2*M; 
Kt = 10*K;
zeta = 0.3;
B = zeta*2*M*omega; 

% Initial conditions
V0 = 0;
Vu0 = 0; 
p2_0 = M*V0;  % initial momentum of top mass 
q6_0 = (M * g) / K;  % initial suspension displacement (equilibrium)
p8_0 = Mu*Vu0;  % initial momentum of tire mass 
q11_0 = ((M + Mu) * g) / Kt;  % initial tire displacement (equilibrium)
initial = [p2_0, q6_0, p8_0, q11_0]; 

tspan = linspace(0,4,2001); 

[t, s] = ode45(@lab2_eqns,tspan,initial);

ext = zeros(length(t),2);
ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab2_eqns(t(i), s(i,:));
end

x = s(:,2) - q6_0; 

acceleration = diff(s(:,1)) ./ diff(t) / M / g;
 
figure('Name','displacements','NumberTitle','off','Color','white')
plot(t, x,'k'), grid on
title('Suspension Deflection')
ylabel('displacement (m)')
xlabel('time (s)')

figure('Name','velocities','NumberTitle','off','Color','white')
plot(t(1:end-1), acceleration,'k'), grid on
title('Mass Acceleration')
ylabel('Acceleration (g)')
xlabel('time (s)')


