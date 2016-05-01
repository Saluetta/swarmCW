function s = initializeSimulatedUAV( )
% It creates the ground truth (real values without noise) of a simulated UAV

    s = [   0 + randn(); % 0 + N(0,1)
            0 + randn(); % 0 + N(0,1)
            rand()*2*pi ]; % Random angle between 0 and 2*pi

end

