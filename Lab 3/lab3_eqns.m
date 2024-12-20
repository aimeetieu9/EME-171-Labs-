function [ds, ext] = lab3_eqns(t, s)
global vC lCG_standard lCG_forward mCR rGY kSF kSR bSF bSR mTF mTR kTF kTR lWB A L g Vc L a b jCR;
    
    % State Variables (lab 3)
    pJ = s(1); 
    pCR = s(2); 
    qSF = s(3); 
    qSR = s(4); 
    pTF = s(5);
    pTR = s(6);
    qTF = s(7);
    qTR = s(8); 

    % Effort Source (from lab 3)
    se_cr = mCR * g;  %gravity on cycle and rider
    se__tf = mTF * g;  %gravity on front tire
    se_tr = mTR * g;  %gravity on rear tire

    % Flow Source (lab 3)
    
    L = 0.5; % m, bump distance
    %A = 0.16; % m, placeholder need to make max height before suspension deflection = 0.1m
   
    T1 = 0.5; %s, time when front tire hits first 
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
  
    Vamp = 2*A*vC/L;

   %setting vFI
    if t < T1
        vFI = 0;
    elseif t >= T1 && t <= T2
        vFI = Vamp;
    elseif t > T2 && t <= T3
        vFI = -Vamp;
    elseif t >= T6 && t <= T7
        vFI = Vamp;
    elseif t > T7 && t <= T8
        vFI = -Vamp;
    else
        vFI = 0;
    end

    %setting vRI
     if t < T4
        vRI = 0;
    elseif t >= T4 && t <= T5
        vRI = Vamp;
    elseif t > T5 && t <= T6
        vRI = -Vamp;
    elseif t >= T9 && t <= T10
        vRI = Vamp;
    elseif t > T10 && t <= T11
        vRI = -Vamp;
    else
        vRI = 0;
    end


    % Equations of Motion (EOM) 
    pJ_dot = (a*((qSF*kSF) + (bSF*((pTF/mTF)-(pCR/mCR)-(a*pJ/jCR))))) - (b*((qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) + (b*pJ/jCR))))); 
    pCR_dot = (-mCR*g) + (qSF*kSF) + (bSF*((pTF/mTF) - (pCR/mCR) -(a*pJ/jCR))) + (qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) +(b*pJ/jCR)));
    qSF_dot = pTF/mTF - pCR/mCR - a*pJ/jCR; 
    qSR_dot = pTR/mTR - pCR/mCR + b*pJ/jCR; %add in 'b' parameter
    pTF_dot = qTF*kTF - mTF*g - qSF*kSF - bSF*(pTF/mTF - pCR/mCR - a*pJ/jCR); %in the parentheses is = qSF_dot
    pTR_dot = qTR*kTR - mTR*g - qSR*kSR - bSR*(pTR/mTR - pCR/mCR + b*pJ/jCR);
    qTF_dot = vFI - pTF/mTF; 
    qTR_dot = vRI - pTR/mTR; 

    % External variables
    ext(1) = vRI; 
    ext(2) = vFI; 
    
    % State derivatives
    ds = [pJ_dot; pCR_dot; qSF_dot; qSR_dot; pTF_dot; pTR_dot; qTF_dot; qTR_dot];
end
