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
figure('units','normalized','outerposition',[0 0 1 1])
hold on;

%% define time and time step
t = 0; % [s]
tMax = 1800; % [s] 30 minutes
dt = 2; % [s]
nSteps = tMax / dt; % simulation steps

%% initialize swarm parameters
nRavens = 2; % number of UAVs

%% initialize true state and control input
X = zeros(3,nRavens); % [m; m; rad]
X = [-100,100;0,0;0,0];

v = 10*ones(1,nRavens); % [m/s]
mu = 0.1*ones(1,nRavens); % [rad/m]
U = [v; mu];

%% initialize agent memory
for i = 1:1:nRavens
    memory(i).lastPosition = X(1:2,i);
    memory(i).stateFSM = 1;
    memory(i).pollution = 0;
    memory(i).turnDirection = 1;
    
%     target(:,i) = [-500*cos(i*2*pi/nRavens);500*sin(i*2*pi/nRavens)];
    target(:,i) = [500,10*i];
end

%% initialize communications
messagePool = initCommunication();

%% target
% target = repmat([500;0],1,nRavens);

%% colors for pretty plotting
colours = [0,0.4470,0.7410;
           0.8500,0.3250,0.0980;
           0.9290,0.6940,0.1250];

%% main simulaiton loop
for k = 1:nSteps
    % update time
    t = t + dt;
    cla
    
    for i = 1:1:nRavens
        % receive broadcasted messages
        receivedMessages = receiveBroadcast(messagePool);
        
        % get estimate of current position from GPS
        Y(i) = simEstimateState(X(:,i), memory(i), target(:,i),receivedMessages, i);

        % agent makes a decision based on its estimated state, y
        [U(:,i),memory(i), target(:,i)] = simDecision(Y(:,i), U(:,i), memory(i), target(:,i), receivedMessages, i);

        % move uav
        X(:,i) = simMove(X(:,i),U(:,i),dt);

        % take measurement
        p(1,i) = cloudsamp(cloud,X(1,i),X(2,i),t);
        memory(i).pollution = p(1,i);

        % add message to broadcast queue
        messageToSend = [X(1,i); X(2,i); p(1,i)];
        messagePool = broadcastMessage(messageToSend, messagePool);

        % drawing
        agentColour = colours(memory(i).stateFSM,:);
        plot(X(1,i),X(2,i),'Color',agentColour,'Marker','o', 'MarkerSize',3) % robot location
        plot(target(1,i), target(2,i), 'sg') % target
    end  
    
    % simulate broadcast of messages
    messagePool = simulateBroadcast(messagePool);
    
    cloudplot(cloud,t)
    
    pause(0.1)
end

end % end of main

%% Helper Functions -------------------------------------------------------
% -------------------------------------------------------------------------
function [ Y ] = simEstimateState( X, memory, target, messages, agentNo)
%SIMESTIMATESTATE simulates estimateion of state based on GPS
%   For the position, adds a gaussian noise of 3m
%   for the heading, uses previous known position and computes arc tangent
%   for heading to target, first find target orientation and then subtract
%   agent's heading
% [Note: heading is measued from North in clockwise direction]

Y.position = X(1:2,1) + 3*randn(2,1);

Y.heading = atan2(Y.position(1,1) - memory.lastPosition(1,1),...
                  Y.position(2,1) - memory.lastPosition(2,1));
              
Y.headingToTarget = bindAngleToFourQuadrant(...
                    atan2(target(1,1) - memory.lastPosition(1,1),...
                          target(2,1) - memory.lastPosition(2,1))...
                    - Y.heading);
                
% % calculate distance to nearest neighbouring agent
% Y.nearestAgentVector = [inf;inf]; % vector from this agent to nearest one
% nearestAgent = [inf;inf]; % estimated position of the nearest agent
% Y.nearestDist = inf;
% % loop through all agents
% for k=1:size(messages,2)
%     if k ~= agentNo % dont check this agent with itself
%         distance = norm(messages(1:2,k) - Y.position(1:2));
%         if (distance < Y.nearestDist)
%             Y.nearestAgentVector = messages(1:2,k) - Y.position(1:2);
%             nearestAgent = messages(1:2,k);
%             Y.nearestDist = distance;
%         end
%     end
% end
% 
% Y.headingToNearestAgent = bindAngleToFourQuadrant(...
%                           atan2(nearestAgent(1) - Y.position(1),...
%                                 nearestAgent(2) - Y.position(2))...
%                           - Y.heading);



end

% -------------------------------------------------------------------------
function [ U_new, memory, target] = simDecision( Y, U, memory, target, messages, agentNo )
%SIMDECISION returns new velocity commands based on current estimated
%state and internal memory

% updae agent's memory
memory.lastPosition = Y.position;

% find agent that is closest to this agent
% calculate distance to nearest neighbouring agent
nearestAgentVector = [inf;inf]; % vector from this agent to nearest one
nearestAgent = [inf;inf]; % estimated position of the nearest agent
nearestDist = inf;
% loop through all agents
for k=1:size(messages,2)
    if k ~= agentNo % dont check this agent with itself
        distance = norm(messages(1:2,k) - Y.position(1:2));
        if (distance < nearestDist)
            nearestAgentVector = messages(1:2,k) - Y.position(1:2);
            nearestAgent = messages(1:2,k);
            nearestDist = distance;
        end
    end
end

headingToNearestAgent = bindAngleToFourQuadrant(...
                          atan2(nearestAgent(1) - Y.position(1),...
                                nearestAgent(2) - Y.position(2))...
                          - Y.heading);




% target = target;
% a finite state machine to decide what control inputs to be given
switch memory.stateFSM
    case 1, % Move to specified target
        % update velocity command based on current heading to target
        v_new = 10 * ((pi/2 - abs(Y.headingToTarget))/(pi/2));
        mu_new =  (3*pi/180) * (Y.headingToTarget/(pi/2));
        
        if nearestDist < 50
            memory.stateFSM = 2;
        else
            if norm(Y.position) > 1000
                memory.stateFSM = 4;
            else
                if memory.pollution > 0.85 && memory.pollution < 1.15
                    memory.stateFSM = 3;
                    target = Y.position;
                end
            end
        end
        
    case 2, % if colliding, evade
        v_new = 10 * ((pi/2 - abs(headingToNearestAgent+pi/2))/(pi/2));
        mu_new = (6*pi/180) * ((headingToNearestAgent+pi/2)/(pi/2));
        
        if nearestDist > 100
            memory.stateFSM = 1;
        end
        
    case 3, % track contour
        v_new = 10;
        mu_new = 6*pi/180;% *  memory.turnDirection;
        memory.turnDirection = memory.turnDirection * -1;
        
        if nearestDist < 150
            memory.stateFSM = 2;
        else
            if memory.pollution > 0.85 && memory.pollution < 1.15
                memory.stateFSM = 3;
                target = Y.position;
            else
                memory.stateFSM = 1;
            end
        end
    case 4, % reaching boundary, turn back
        v_new = 10 * ((pi/2 - abs(Y.headingToTarget - pi))/(pi/2));
        mu_new =  (3*pi/180) * (Y.headingToTarget - pi)/(pi/2);
        
        if nearestDist < 150
            memory.stateFSM = 2;
        else
            if norm(Y.position) > 1000
               memory.stateFSM = 1; 
            end
        end
end

[v_new, mu_new] = applyConstraints(v_new, mu_new);
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

function messagePool = initCommunication()
% initialize communication
% curretnMessages are read by agents
% new messages are collected in simulation loop and replace current ones
    messagePool.currentMessages = [];
    messagePool.newMessages = [];
end

function messages = receiveBroadcast(messagePool)
% receive all messages broadcast
    messages = messagePool.currentMessages;
end

function messagePool = broadcastMessage(message, messagePool)
% send message - adds message to the set of new messages
    messagePool.newMessages = [messagePool.newMessages message];
end

function messagePool = simulateBroadcast(messagePool)
% replace set of current messages with new set
    messagePool.currentMessages = messagePool.newMessages;
    messagePool.newMessages = [];
end

% Math Functions ----------------------------------------------------------
function angle = bindAngleToFourQuadrant(angle)
% input angle in radians
% sets range from -pi to pi
    if angle > pi
        angle = angle - 2*pi;
    elseif angle < -pi
        angle = angle + 2*pi;
    end
end