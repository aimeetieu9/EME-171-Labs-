%calculating max time step size (number of steps)
function [numofsteps] = lab3_timestepcalc(starttime, endtime)
global M K B amp Vc L  Mu Kt maxTimeStep vC lCG_standard lCG_forward mCR rGY kSF kSR bSF bSR mTF mTR kTF kTR lWB A L g Vc L A;

    T1 = 0; %change to alter starting value of pothole
    T2 = T1 + L/(2*Vc);

%call lab2_eqns to get global values needed for calcs here:


%max time step calc FEEL FREE TO CHECK THESE CALCS:
    dThalfstep = T2 - T1; % time to traverse half pothole
    maxStepHalfPot = dThalfstep/10; % max step for half 10 points per half pothole
    
    %highest natural freq is:
    omegan1 = sqrt(K/M);
    omegan2 = sqrt(Kt/Mu);
    period1 = 1/omegan1; % = cycle time 1
    period2 = 1/omegan2; % = cycle time 2
    
    if omegan1 >= omegan2
        maxStepfromFreq = period1/10;
    else
        maxStepfromFreq = period2/10;
    end
    
    if maxStepfromFreq <= maxStepHalfPot
        maxTimeStep = maxStepfromFreq;
    else
        maxTimeStep = maxStepHalfPot; % sec/step
    end

    numofsteps = (endtime-starttime)/maxTimeStep;
   

end