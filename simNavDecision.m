function [ U_new, navMemory ] = simNavDecision( Y, U, navMemory )
%SIMNAVDECISION returns new velocity commands based on current estimated
%state and internal memory

% updae agent's memory
navMemory.lastPosition = Y.position;
navMemory.velocityCommands = U;

% update velocity command based on current heading to target
U_new(1,1) = 10 * ((pi/2 - abs(Y.headingToTarget))/(pi/2));
U_new(2,1) =  (3*pi/180) * (Y.headingToTarget/(pi/2));

end

