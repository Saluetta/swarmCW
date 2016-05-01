function sim_multiple_uav
% author: manaswi
% description: simulation of multiple uavs moving to specified targets and
% circling nearby

%% tabula rasa
% clear all
close all
clc

%% load cloud data
load 'cloud1.mat'
% load 'cloud2.mat'

%% initialize figure
figure
hold on;

%% define time and time step
t = 0; % [s]
tMax = 1800; % [s] 30 minutes
dt = 2; % [s]
nSteps = tMax / dt; % simulation steps

%% initialize swarm parameters
nRavens = 5; % number of UAVs

%% initialize true state and control input
X = zeros(3,nRavens); % [m; m; rad]

v = 10*ones(1,nRavens); % [m/s]
mu = 0.1*ones(1,nRavens); % [rad/m]
U = [v; mu];

%% initialize agent memory
for i = 1:1:nRavens
    memory(i).lastPosition = X(1:2,1);
    % memory.velocityCommands = U;
    memory(i).stateFSM = 1;
    target(:,i) = [500*cos(i*2*pi/nRavens);500*sin(i*2*pi/nRavens)];
end

%% target
% target = repmat([500;0],1,nRavens);

%% main simulaiton loop
for k = 1:nSteps
    % update time
    t = t + dt;
    cla
    
    for i = 1:1:nRavens
        % get estimate of current position from GPS
        Y(i) = simEstimateState(X(:,i), memory(i), target(:,i));

        % agent makes a decision based on its estimated state, y
        [U(:,i),memory(i)] = simDecision(Y(:,i), U(:,i), memory(i));

        % move uav
        X(:,i) = simMove(X(:,i),U(:,i),dt);

        % take measurement
        p(1,i) = cloudsamp(cloud,X(1,i),X(2,i),t);

        % adjust target based on measurement
        if p(1,i) > 0.85 && p(1,i) < 1.15
            target = X(1:2,i);
            memory(i).stateFSM = 2;
        end

        % drawing
        plot(X(1,i),X(2,i),'o') % robot location
        plot(target(1,i), target(2,i), 'sg') % target
    end
    
    
    % drawing
%     
%     title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, X(1,1),X(2,1),p)) 
    
    
    cloudplot(cloud,t)
    
    pause(0.1)
end

end % end of main

%% Helper Functions -------------------------------------------------------
% -------------------------------------------------------------------------
function [ Y ] = simEstimateState( X, memory, target )
%SIMESTIMATESTATE simulates estimateion of state based on GPS
%   For the position, adds a gaussian noise of 3m
%   for the heading, uses previous known position and computes arc tangent
%   for heading to target, first find target orientation and then subtract
%   agent's heading

Y.position = X(1:2,1) + 3*randn(2,1);

Y.heading = atan2(Y.position(1,1) - memory.lastPosition(1,1),...
                  Y.position(2,1) - memory.lastPosition(2,1));
              
Y.headingToTarget = atan2(target(1,1) - memory.lastPosition(1,1),...
                          target(2,1) - memory.lastPosition(2,1))...
                    - Y.heading;

% [Note: heading is measued from North in clockwise direction]
end

% -------------------------------------------------------------------------
function [ U_new, memory ] = simDecision( Y, U, memory )
%SIMDECISION returns new velocity commands based on current estimated
%state and internal memory

% updae agent's memory
memory.lastPosition = Y.position;
% memory.velocityCommands = U;

% a finite state machine to decide what control inputs to be given
switch memory.stateFSM
    case 1, % Move to specified target
        % update velocity command based on current heading to target
        v_new = 10 * ((pi/2 - abs(Y.headingToTarget))/(pi/2));
        mu_new =  (3*pi/180) * (Y.headingToTarget/(pi/2));
    
    case 2, % If reached target, circle nearby
        v_new = 20;
        mu_new = 1.5*pi/180;
end

[v_new,mu_new] = applyConstraints(v_new,mu_new);


U_new = [v_new; mu_new];

end

function [v_new, mu_new] = applyConstraints(v_new,mu_new)

% apply limits on v
if v_new > 20
    v_new = 20;
end

if v_new < 10
    v_new = 10;
end

% apply limits on mu
if mu_new > 6*pi/180
    mu_new = 6*pi/180;
end

% TODO: check this
% if mu_new < 0
%     mu_new = 0;
% end

end


% -------------------------------------------------------------------------
function [ X_next ] = simMove( X,U,dt )
%simMove given current state, control input and time step, this function
%returns the state at the next instant of time
%   Implements a simple 4th order Runge Kutta prediction

k1 = continuousDynamics(X,U);
k2 = continuousDynamics(X+k1*dt/2,U);
k3 = continuousDynamics(X+k2*dt/2,U);
k4 = continuousDynamics(X+k3*dt,U);

X_next = X + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
X_next(3,1) = mod(X_next(3,1), 2*pi);

end

% -------------------------------------------------------------------------
function [ X_dot ] = continuousDynamics( X,U )
%CONTINUOUSDYNAMICS simulates continuous dynamics of the system
%   Taken from the model of the UAV
%   X = [x;y;theta], U = [v;mu]
%   x' = v sin(theta)
%   y' = v cos(theta)
%   theta' = v mu

X_dot = zeros(3,1);
X_dot(1,1) = U(1,1) * sin( X(3,1) );
X_dot(2,1) = U(1,1) * cos( X(3,1) );
X_dot(3,1) = U(1,1) * U(2,1);

end



% -------------------------------------------------------------------------
% Communication Functions

function channel = initChannel()
% initialize comms channel model
    channel.curMsgs = {};
    channel.newMsgs = {};
end

function [rxMsgs,channel] = simReceive(aa,channel)
% simulate receiving messages
% simple broadcast model - just get everything
    rxMsgs = channel.curMsgs;
end

function channel = simTransmit(txMsgs,aa,channel)
% simulate transmitting a message
% store it for next step
    channel.newMsgs = [channel.newMsgs txMsgs];
end

function channel = simChannel(channel,x)
% simple - everyone gets everything
    channel.curMsgs = channel.newMsgs;
    channel.newMsgs = {};
end