function [ Y ] = simGPS( X, navMemory, target )
%SIMGPS simulates GPS measurements
%   For the position, adds a gaussian noise of 3m
%   for the heading, uses previous known position and computes arc tangent
%   for heading to target, first find target orientation and then subtract
%   agent's heading

Y.position = X(1:2,1) + 3*randn(2,1);
Y.heading = atan2(Y.position(2,1) - navMemory.lastPosition(2,1),...
                  Y.position(1,1) - navMemory.lastPosition(1,1));
Y.headingToTarget = atan2(target(2,1) - navMemory.lastPosition(2,1),...
                          target(1,1) - navMemory.lastPosition(1,1))...
                    - Y.heading;

end

