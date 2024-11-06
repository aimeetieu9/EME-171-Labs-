clear all;
close all;
clc;
global a b vC jCR lCG_standard lCG_forward mCR rGY kSF kSR bSF bSR mTF mTR kTF kTR lWB A L g;


%parameters
amp = 0.1; %m/s, velocity input (road profile)

% %system parameters MODIFIED!
vC = 10;
lCG_standard = 0.9; 
lCG_forward = 0.7; 
mCR = 300; 
rGY = 0.5; 
kSF = 3000; %front sus stiffness
kSR = 3500; %rear sus stiffness
bSF = 400; 
bSR = 500; 
mTF = 15; 
mTR = 20; 
kTF = 30000; %front tire stiffness
kTR = 40000; %rear tire stiffness
lWB = 1.6; 
A = 0.1665; %bump height CHANGE!!!!!!!!!!
L = 0.5; 
g = 9.81; %m/s^2;
jCR = mCR * rGY^2; %rotational inertia

% Initial conditions
%b = lWB - lCG_forward;
%a = lCG_forward;
b = lWB - lCG_forward;
a = lCG_forward;

vFI = 0; 
vRI = 0; 
p_J0 = 0;    
p_cr0 = 0;  
p_tf0 = 0;  
p_tr0 = 0;   
q_tf0 =  ((mTF * g)  + (b*mCR*g) / (b+a)) / kTF;
q_tr0 = ((mTR * g) + ((mCR*g)/((b/a) + 1))) / kTR;
q_sf0 = (b*mCR*g) / ((b+a) * kSF);
q_sr0 = (mCR * g) / (((b/a) + 1) * kSR);
initial = [p_J0, p_cr0, q_sf0, q_sr0, p_tf0, p_tr0, q_tf0, q_tr0]; 

% % TIME STEP CALC
% % shortest vibration period
% Trw = 2*pi / sqrt((kTR)/jCR);
% Tfw = 2*pi / sqrt(kTF/jCR);
% Theave = 2*pi / sqrt((kSR+kSF)/jCR);
% Tpitch = (2*pi) / sqrt(kSR/jCR) * b^2 + (2*pi) / sqrt(kSF/jCR) * a^2;
% 
% Frw = 1/Trw * 2*pi; %natural frequency [rad/s]
% Ffw = 1/Tfw * 2*pi;
% Fheave = 1/Theave * 2*pi;
% Fpitch = 1/Tpitch * 2*pi;

TIME STEP CALC
% shortest vibration period
% Trw = 2*pi / sqrt((kTR)/jCR);
% Tfw = 2*pi / sqrt(kTF/jCR);
% Theave = 2*pi / sqrt((kSR+kSF)/jCR);
% Tpitch = (2*pi) / sqrt(kSR/jCR) * b^2 + (2*pi) / sqrt(kSF/jCR) * a^2;

Frw = sqrt((kTR)/mTR); %natural frequency [rad/s]
Ffw = sqrt(kTF/mTF);
Fheave = sqrt((kSR+kSF)/mCR);
Fpitch = sqrt( (kSF*(a^2)) + (kSR*(b^2))/jCR);


Tmin = min([Trw,Tfw,Theave,Tpitch]);
Tmax = max([Trw,Tfw,Theave,Tpitch]);
%time it takes to go over 1/2 bump
    T1 = 0; %s, time when front tire hits first 
    T2 = T1 + L/(2*vC); %s, front tire apex 1st 
    T3 = T1 + L/vC; %s, front tire end first 
    T4 = T1 + lWB/vC; %s, back tire start 1st 
    T5 = T4 + L/(2*vC); %s, back tire apex 1st 
    T6 = T4 + L/vC; %s, back tire end 1st & front tire start 2nd
    T7 = T6 + L/(2*vC); %s, front tire apex 2nd
    T8 = T6 + L/vC; %s, front tire end 2nd 
    T9 = T6 + lWB/vC; %s, back tire start 2nd 
    T10 = T9 + L/(2*vC); %s, back tire 2nd apex
    T11 = T9 + L/vC; %s, back tire end 2nd
Thalfbump = L/(2*vC);
maxstepsize = min(Tmin/10, Thalfbump/10);

tspanstart = 0;
tspanend = 3*Tmax + T11; % 3 x Tmax + time tire reaches end of second bump
numofsteps = (tspanend-tspanstart)/maxstepsize;
tspan = linspace(tspanstart,tspanend,numofsteps);

[t, s] = ode45(@lab3_eqns,tspan,initial);

ext = zeros(length(t),2);
ds = zeros(length(t),8);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab3_eqns(t(i), s(i,:));
end

sf_deflection = s(:,3) - q_sf0;
sr_deflection = s(:,4) - q_sr0;
max(sr_deflection)

%FRONT AND REAR IDENTICAL BUT SHIFTED BY A TIME
% Front and Rear Suspension Deflections 
figure('Name','Suspension Deflection','NumberTitle','off','Color','white')
plot(t, sf_deflection,'k',t, sr_deflection, 'r'), grid on
title('Suspension Deflection')
ylabel('displacement (m)')
xlabel('time (s)')
legend('Front Suspension', 'Rear Suspension')

% Heave Velocity
v_heave = s(:,2)/mCR; 
figure('Name','Heave Velocity','NumberTitle','off','Color','white')
plot(t, v_heave,'k'), grid on
title('Heave Velocity')
ylabel('velocity (m/s)')
xlabel('time (s)')

%Pitch Angular Velocity 
omega_velocity= s(:,1)/jCR; 
figure('Name','Pitch Angular Velocity','NumberTitle','off','Color','white')
plot(t, omega_velocity, 'k'), grid on
title('Pitch Angular Velocity')
ylabel('Pitch Angular Velocity (rad/s)')
xlabel('time (s)')

%Plotting velocity inputs to check
figure('Name', 'Road Velocity Check','NumberTitle','off', 'Color','white')
plot(t, ext(:,2), 'b', t, ext(:,1), 'r'), grid on
title('Road Velocity Check')
ylabel('velocity (m/s)')
xlabel('time (s)')
legend('Front Input Velocity', 'Rear Input Velocity')
