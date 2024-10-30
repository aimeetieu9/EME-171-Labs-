clear all;
close all;
clc;
global vC lCG_standard lCG_forward mCR rGY kSF kSR bSF bSR mTF mTR kTF kTR lWB A L g;


%parameters
amp = 0.1; %m/s, velocity input (road profile)

%system parameters MODIFIED! 
vC = 10; 
lCG_standard = 0.9; 
lCG_forward = 0.7; 
mCR = 300; 
rGY = 0.5; 
kSF = 3000;
kSR = 3500; 
bSF = 400; 
bSR = 500; 
mTF = 15; 
mTR = 20; 
kTF = 30000; 
kTR = 40000;
lWB = 1.6; 
A = 0.1;%bump height CHANGE!!!!!!!!!!
L = 0.5; 
g = 9.81; %m/s^2;
jCR = mCR * rGY^2; 

% INITIAL CONDITIONS CALCULATION (set derivatives = 0):
% for standard conditions, a = lCG_standard, b = lWB - lCG_standard
% for forward conditions, a = lCG_forward, b = lWB - lCG_forward
    
    % eqn1 = 0 == (a*((qSF*kSF) + (bSF*((pTF/mTF)-(pCR/mCR)-(a*pJ/jCR))))) - (b*((qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) + (b*pJ/jCR))))); 
    % eqn2 = 0 == (-mCR*g) + (qSF*kSF) + (bSF*((pTF/mTF) - (pCR/mCR) -(a*pJ/jCR))) + (qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) +(b*pJ/jCR)));
    % eqn3 = 0 == pTF/mTF - pCR/mCR - a*pJ/jCR; %a is dependent on forward or standard, add parameter (not the same as A, bump height)
    % eqn4 = 0 == pTR/mTR - pCR/mCR + b*pJ/jCR; %add in 'b' parameter
    % eqn5 = 0 == qTF*kTF - mTF*g - qSF*kSF - bSF*(pTF/mTF - pCR/mCR - a*pJ/jCR); %in the parentheses is = qSF_dot
    % eqn6 = 0 == qTR*kTR - mTR*g - qSR*kSR - bSR*(pTR/mTR - pCR/mCR + b*pJ/jCR);
    % eqn7 = 0 == vFI - pTF/mTF; %add in vFI
    % eqn8 = 0 == vRI - pTR/mTR; %add in vRI
    % 
    % Initialeqns = [eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8];
    % InitialVar = [pJ,pCR,qSF,qSR,pTF,pTR,qTF,qTR];
    % 
    % sol = solve(Initialeqns,InitialVar);
    %CHECKKKKK

% Initial conditions
b = lwb - lCG_forward;
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

% time step
tspanstart = 0;
tspanend=4;
numofsteps = lab3_timestepcalc(tspanstart, tspanend);
tspan = linspace(tspanstart,tspanend,numofsteps); %CHANGE!!!!!!!

[t, s] = ode45(@lab3_eqns,tspan,initial);

ext = zeros(length(t),2);
ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab3_eqns(t(i), s(i,:));
end


sf_deflection = q_sf0 - s(:,3);
sr_deflection = q_sr0 - s(:,4);
%FRONT AND REAR IDENTICAL BUT SHIFTED BY A TIME
% Front and Rear Suspension Deflections 
figure('Name','Suspension Deflection','NumberTitle','off','Color','white')
plot(t, sf_deflection,'k',t, sr_deflection, 'r', t, s(:,9), 'c'), grid on
title('Suspension Deflection')
ylabel('displacement (m)')
xlabel('time (s)')

% Heave Velocity
v_heave = s(:,2)/mCR; 
figure('Name','Heave Velocity','NumberTitle','off','Color','white')
plot(t, v_heave,'k', t, s(:,9), 'k'), grid on
title('Heave Velocity')
ylabel('velocity (m/s)')
xlabel('time (s)')

%Pitch Angular Velocity 
omega_velocity= s(:,1)/jCR; 
figure('Name','Pitch Angular Velocity','NumberTitle','off','Color','white')
plot(t, s(:,9),'k', t, omega_velocity, 'k'), grid on
title('Heave Velocity')
ylabel('Pitch Angular Velocity (rad/s)')
xlabel('time (s)')



