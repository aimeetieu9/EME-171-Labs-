function [ds, ext] = lab5_eqns(t,s)
global Rw Lw Tm M btau R Gr Cr Cd rho Af Uin g

%State variables
pL = s(1);
pM = s(2);
% Approximation for signum function
sgn_approx = pM / (abs(pM) + (1*10^-5));
% Equations of Motion (EOM) (lab 5)
pLdot = Uin - ((pL*Rw)/Lw) - ((pM*Tm*R)/(Gr*M));
pMdot = (R/Gr)*((Tm*pL/Lw) - ((btau*pM*R/M/Gr))) - 0.5*rho*Af*Cd*(pM/M)*abs(pM/M) - M*g*Cr*sgn_approx;
ds = [pLdot; pMdot];
end
