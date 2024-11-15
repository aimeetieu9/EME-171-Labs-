function [ds, ext] = lab3_eqns(t, s)
global rho g Ap P Cf L Ip At Ct Q0 d0 d1 d2;
    
    % State Variables (lab 4)
    p3 = s(1); 
    p9 = s(2); 
    q7 = s(3); 
    q13 = s(4);

    % Effort Source (from lab 3)
    se1 = rho*g*d0;  
    se6 = rho*g*d1;  
    se12 = rho*g*d2; 
    sf = QO_t; % changes with time MODIFYYYYYYYYYY

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


    % Equations of Motion (EOM) (lab 4)
    qdot7 = (p3/Ip) - (p9/Ip); 
    qdot13 = (p9/Ip) - Q0;
    pdot3 = (rho*g*d0) + (rho*g*d1) - (Cf* (p3/Ip) * abs(p3/Ip)) - (q7/Ct);
    pdot9 = (q7/Ct) + (rho*g*d2) + (q13/Ct) - (Cf*(p9/Ip)*abs(p9/Ip));


    % External variables
    ext(1) = vRI; 
    ext(2) = vFI; 
    
    % State derivatives (lab 4)
    ds = [qdot7; qdot13; pdot3; pdot9];
end
