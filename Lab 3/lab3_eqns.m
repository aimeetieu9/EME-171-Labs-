function [ds, ext] = eqns2(t, s)
global vC lCG_standard lCG_forward mCR rGY kSF kSR bSF bSR mTF mTR kTF kTR lWB A L g;
    
    % State Variables 
    pJ = s(1); 
    pCR = s(2); 
    qSF = s(3);
    qSR = s(4);
    pTF = s(5);
    pTR = s(6);
    qTF = s(7);
    qTR = s(8); 

    % Effort Source 
    se1 = M*g;
    se9 = Mu *g;

    % Flow Source
    Vc = 10; % m/s
    L = 1.2; % m
    A = 0.08; % m
    T1 = 0; %change to alter starting value
    T2 = T1 + L/(2*Vc);
    T3 = T1 + L/Vc;
    Vamp = 2*A*Vc/L;

    if t < T1
        Vi = 0;
    elseif t >= T1 && t <= T2
        Vi = Vamp;
    elseif t > T2 && t <= T3
        Vi = -Vamp;
    else
        Vi = 0;
    end

    % Equations of Motion (EOM) 
    pJ_dot = (a*((qSF*kSF) + (bSF*((pTF/mTF)-(pCR/mCR)-(a*pJ/jCR))))) - (b*((qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) + (b*pJ/jCR))))); 
    pCR_dot = (-mCR*g) + (qSF*kSF) + (bSF*((pTF/mTF) - (pCR/mCR) -(a*pJ/jCR))) + (qSR*kSR) + (bSR*((pTR/mTR) - (pCR/mCR) +(b*pJ/jCR)));
    qSF_dot = pTF/mTF - pCR/mCR - a*pJ/jCR; %is 'a' the same as 'A' ????
    qSR_dot = pTR/mTR - pCR/mCR + b*pJ/jCR; %add in 'b' parameter
    pTF_dot = qTF*kTF - mTF*g - qSF*kSF - bSF*(pTF/mTF - pCR/mCR - a*pJ/jCR); %in the parentheses is = qSF_dot
    pTR_dot = qTR*kTR - mTR*g - qSR*kSR - bSR*(pTR/mTR - pCR/mCR + b*pJ/jCR);
    qTF_dot = vFI - pTF/mTF; %add in vFI
    qTR_dot = vRI - pTR/mTR; %add in vRI

    droad_dot = Vi;

    % External forces
    ext(1) = B * (Vi / M); 
    ext(2) = Vi; 
    
    % State derivatives
    ds = [pJ_dot; pCR_dot; qSF_dot; qSR_dot; pTF_dot; pTR_dot; qTF_dot; qTR_dot]; % modified 
end
