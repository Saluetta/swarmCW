function sim_single_uav
% author: manaswi
% description: simulation of single uav

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
dt = 3.6; % [s]

%% initialize state and control input
X = zeros(3,1); % [m; m; rad]

v = 10; % [m/s]
mu = 0.1; % [rad/s]
U = [v; mu];

%% main simulaiton loop
for k = 1:1000 % 1000 steps
    t = t+dt;
    
    % move uav
    X = move(X,U,dt);
    
    % take measurement
    p = cloudsamp(cloud,X(1,1),X(2,1),t);
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, X(1,1),X(2,1),p))
        
    % plot robot location
    plot(X(1,1),X(2,1),'o')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
end