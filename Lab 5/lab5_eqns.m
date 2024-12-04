% % %% Part 1
% function [ds] = lab5_eqns(t,s)
% global Rw Lw Tm M btau R Gr Cr Cd rho Af Uin g
% 
% %State variables
% pL = s(1);
% pM = s(2);
% % Approximation for signum function
% sgn_approx = pM / (abs(pM) + (1*10^-5));
% % Equations of Motion (EOM) (lab 5)
% pLdot = Uin - (pL*Rw)/Lw - (Gr*pM*Tm)/(R*M);
% pMdot = (Gr/R)*((Tm*pL/Lw) - (pM*btau/M)) - M*g*Cr*sgn_approx - 0.5*rho*Af*Cd*(pM/M)*(pM/M);
% ds = [pLdot; pMdot];
% end
%% 
% %% Part 2 
% function [ds, ext] = lab5_eqns(t,s)
% global Rw Lw Tm M btau R Gr Cr Cd rho Af Uin g vref dref uin kp ki
% %State variables
% pL = s(1);
% pM = s(2);
% dref= s(3);
% dact= s(4);
% 
% % Approximation for signum function (From Part 1)
% sgn_approx = pM / (abs(pM) + 1e-5);
% 
% 
% T1 = 0; %Chosen Start Time 
% if t<0
%     vref = 0;
% else
%     vref = 1;
% end
% % Input Velocity (given) 
% uin = kp*(vref-pM/M) + ki*(dref-dact);
% 
% % Equations of Motion (EOM) (lab 5)
% pLdot = uin - (pL*Rw)/Lw - (Gr*pM*Tm)/(R*M);
% pMdot = (Gr/R)*((Tm*pL/Lw) - (pM*btau/M)) - M*g*Cr*sgn_approx - 0.5*rho*Af*Cd*(pM/M)*(pM/M);
% d_refdot = vref;
% ddot = pM/M;
% 
% ds = [pLdot; pMdot; d_refdot; ddot];
% ext=[vref];
% end

%% Part 3
function [ds, ext] = lab5_eqns(t,s)
global Rw Lw Tm M btau R Gr Cr Cd rho Af Uin g vref dref kp ki
%Extract the state variables
pL = s(1);
pM = s(2);
dref= s(3);
dact= s(4);

% Approximation for signum function (From Part 1)
sgn_approx = pM / (abs(pM) + 1e-5);

vref = LA92Oracle(t);

% Input Velocity (given) 
Uin = kp*(vref-pM/M) + ki*(dref-dact);

% Equations of Motion (EOM) (lab 5)
pLdot = Uin - (pL*Rw)/Lw - (Gr*pM*Tm)/(R*M);
pMdot = (Gr/R)*((Tm*pL/Lw) - (pM*btau/M)) - M*g*Cr*sgn_approx - 0.5*rho*Af*Cd*(pM/M)*(pM/M);
d_refdot = vref;
ddot = pM/M;


ds = [pLdot; pMdot; d_refdot; ddot];
ext = [vref];
end