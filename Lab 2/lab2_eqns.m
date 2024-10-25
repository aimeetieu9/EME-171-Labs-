function [ds, ext] = eqns2(t, s)
global M K B amp Vc L A Mu Kt fn omega zeta q6_0 g;
    
    p2 = s(1); % momentum of top mass
    q6 = s(2); % m, suspension displacement
    p8 = s(3); % momenetum of tire mass
    q11 = s(4); % m, tire displacement

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
    % p2 = momentum of top mass | q6 = sus displacement 
    % p8 = momenetum of tire mass | q11 = tire displacement 
    p_dot_2 = (-B/M*p2) - (K*q6) + (B/Mu*p8) + (M*g);
    q_dot_6 = (p2/M) - (p8/Mu);
    p_dot_8 = (B/M*p2) + (K*q6) - (B/Mu*p8) - (Kt*q11) + (Mu*g);
    q_dot_11 = (p8/Mu) - Vi;
    
    droad_dot = Vi;
    % External forces
    ext(1) = B * (Vi / M); 
    ext(2) = Vi; 
    
    % State derivatives
    ds = [p_dot_2; q_dot_6; p_dot_8; q_dot_11];
end
