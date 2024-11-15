function [ds, ext] = lab3_eqns(t, s)
global rho g Ap P Cf L Ip At Ct Q0 d0 d1 d2;
     
    % State Variables (lab 4)
    p3 = s(1); 
    p9 = s(2); 
    q7 = s(3); 
    q13 = s(4);

    % Effort Source (from lab 4)
    se1 = rho*g*d0;  
    se6 = rho*g*d1;  
    se12 = rho*g*d2; 

    % Flow Source (lab 4)
   
    T1 = 0; 
    T2 = 2; %modify, chosen time 
    T3 = T2 + 0.15;  

   %setting Q0
    if t >= T1 && t <= T2
        Q0 = 1.5;
    elseif t > T2 && t <= T3
        Q0 = 1.5 - (10*(t-T2));
    elseif t > T3
        Q0 = 0;
    end

    % Equations of Motion (EOM) (lab 4)
    qdot7 = (p3/Ip) - (p9/Ip); 
    qdot13 = (p9/Ip) - Q0;
    pdot3 = (rho*g*d0) + (rho*g*d1) - (Cf* (p3/Ip) * abs(p3/Ip)) - (q7/Ct);
    pdot9 = (q7/Ct) + (rho*g*d2) - (q13/Ct) - (Cf*(p9/Ip)*abs(p9/Ip));
    


    % External variables
    ext(1) = Q0;
    
    % State derivatives (lab 4)
    ds = [pdot3; pdot9; qdot7; qdot13];
end
