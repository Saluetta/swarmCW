function sim_single_uav
% author: manaswi
% description: simulation of single uav moving to a specified target

%% tabula rasa
clear all
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
dt = 3.6; % [s]
nSteps = tMax / dt;

%% initialize state and control input
X = zeros(3,1); % [m; m; rad]

v = 10; % [m/s]
mu = 0.1; % [rad/s]
U = [v; mu];

%% initialize navigation memory
navMemory.lastPosition = X(1:2,1);
navMemory.velocityCommands = U;
navMemory.state = 1;

%% target
target = [500;0];

%% main simulaiton loop
for k = 1:nSteps
    % update time
    t = t + dt;
    
    % get estimate of current position from GPS
    Y = simGPS(X, navMemory, target);
    
    % agent makes a decision based on its estimated state, y
    [U,navMemory] = simNavDecision(Y, U, navMemory);
    
    % move uav
    X = simMove(X,U,dt);
    
    % take measurement
    p = cloudsamp(cloud,X(1,1),X(2,1),t);
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, X(1,1),X(2,1),p))
        
    % plot 
    plot(X(1,1),X(2,1),'o') % robot location
    plot(target(1,1), target(2,1), 'sg') % target
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
end