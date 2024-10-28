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
A = %bump height
L = 0.5; 
g = 9.81; %m/s^2;
jCR = mCR * rGY^2; 

% INITIAL CONDITIONS CALCULATION (set derivatives = 0):
% for standard conditions, a = lCG_standard, b = lWB - lCG_standard
% for forward conditions, a = lCG_forward, b = lWB - lCG_forward
    
    eqn1 = 0 == (a*((qSF*kSF) + (bSF*((pTF/mTF)-(pCR/mCR)-(a*pJ/jCR))))) - (b*((qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) + (b*pJ/jCR))))); 
    eqn2 = 0 == (-mCR*g) + (qSF*kSF) + (bSF*((pTF/mTF) - (pCR/mCR) -(a*pJ/jCR))) + (qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) +(b*pJ/jCR)));
    eqn3 = 0 == pTF/mTF - pCR/mCR - a*pJ/jCR; %a is dependent on forward or standard, add parameter (not the same as A, bump height)
    eqn4 = 0 == pTR/mTR - pCR/mCR + b*pJ/jCR; %add in 'b' parameter
    eqn5 = 0 == qTF*kTF - mTF*g - qSF*kSF - bSF*(pTF/mTF - pCR/mCR - a*pJ/jCR); %in the parentheses is = qSF_dot
    eqn6 = 0 == qTR*kTR - mTR*g - qSR*kSR - bSR*(pTR/mTR - pCR/mCR + b*pJ/jCR);
    eqn7 = 0 == vFI - pTF/mTF; %add in vFI
    eqn8 = 0 == vRI - pTR/mTR; %add in vRI

    Initialeqns = [eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8];
    InitialVar = [pJ,pCR,qSF,qSR,pTF,pTR,qTF,qTR];

    sol = solve(Initialeqns,InitialVar);
    %CHECKKKKK

% Initial conditions
V0 = 0;
Vu0 = 0; 
p2_0 = M*V0;  % initial momentum of top mass 
q6_0 = (M * g) / K;  % initial suspension displacement (equilibrium)
p8_0 = Mu*Vu0;  % initial momentum of tire mass 
q11_0 = ((M + Mu) * g) / Kt;  % initial tire displacement (equilibrium)
initial = [pJ_0, pCR, qSF, qSR, pTF, pTR, qTF, qTR]; 

tspan = linspace(0,4,2001); %CHANGE!!!!!!!

[t, s] = ode45(@lab3_eqns,tspan,initial);

ext = zeros(length(t),2);
ds = zeros(length(t),4);

for i = 1:length(t)
[ds(i,:) ext(i,:)] = lab3_eqns(t(i), s(i,:));
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


