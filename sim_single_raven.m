function sim_single_raven
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud1.mat'

% time and time step
t = 0;
dt = 3.6;

% initial dynamic state [pos;vel] in 2D
x = [0;0;0;
    zeros(3,1)];

% v units -> m/s || mu units -> rad/s

v = 10;
mu = 0;

u = [v;mu];
% initial decision maker memory
% last recorded position
navMemory.lastPos = [0;0]; 
% estimate of own velocity
%navMemory.velEstimate = [v*sin(0);v*cos(0);v*mu];
navMemory.velCommands = [u(1),u(2)];
% time step
%dt = 0.1;

% target position
targ = [500;0];

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
    y = sim_GPS(x,navMemory,targ);
    
    % agent makes decision on acceleration
    %u = [1.2;1.2;6*pi/180];%simNavDecision(y,kk,navMemory);
    
    [u,navMemory] = simNavDecision(y,u,navMemory);
    % cheat - robot goes round in circles
    % x = [500*cos(0.01*t); 500*sin(0.01*t); 0.2 ] ;
    % execute decision
    x = simMove(x,u,navMemory,dt);
    
    % take measurement
    p = cloudsamp(cloud,x(1),x(2),t);
    
    if p > 0.8 && p < 1.2
        targ = y.Position;
    end
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),p))
        
    % plot robot location
    plot(x(1),x(2),'o')
    plot(targ(1),targ(2),'*')
    %plot(navMemory.lastPos(1),navMemory.lastPos(2),'g+')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    % pause ensures that the plots update
    pause(0.1)
    
end


% For dynamics> 1.90509 kg 
% Governing equations: x_dot = v * sin(theta)
%                      y_dot = v * cos(theta)
%                      theta_dot = v * mu

function [u, navMemory] = simNavDecision(y,u,navMemory)
% simulate agent deciding own acceleration

% first get updated velocity estimate
% start with direct differencing from last time
%%%noisyVel = (y.ownPosition - navMemory.lastPos)/0.1;
% and filter to remove some of the noise
%%%navMemory.velEstimate = navMemory.velEstimate + ...
%%%                 0.2*(noisyVel - navMemory.velEstimate);
% reset last position store with new measurement
navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];

% guidance - constant speed towards target
%targVel = y.targetVector*0.1/norm(y.targetVector);
%y.HeadingToGoal
u(1) = 15 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
u(2) =  (6*pi/180) * (y.HeadingToGoal/(pi/2));

% control - accelerate to that velocity
%u = 0.3*(targVel - navMemory.velEstimate);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xnew=simMove(x,u,navMemory,dt)
% runge kutta 4th order
k1 = f_continuous(x,u,navMemory);
k2 = f_continuous(x+k1*dt/2,u,navMemory);
k3 = f_continuous(x+k2*dt/2,u,navMemory);
k4 = f_continuous(x+k3*dt,u,navMemory);
xnew = x+(k1+2*k2+2*k3+k4)*dt/6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot=f_continuous(x,u,navMemory)
% simple point mass: x is [2D pos; 2D vel] and u is [2D acc]
% so xdot is just a rearrange
%xdot = [x(3:4); u];
xdot = [u(1)*sin(x(3));...
        u(1)*cos(x(3));...
        u(1)*u(2);...
        (navMemory.velCommands(1)-u(1))*sin(x(3));...
        (navMemory.velCommands(1)-u(1))*cos(x(3));...
        (navMemory.velCommands(1)-u(1))*(navMemory.velCommands(2)-u(2))];

function y = sim_GPS(x,navMemory,targ)
% simulate measurement for agent aa
% measurement in three parts

% first is position, plus noise
y.Position = x(1:2) + 0.01*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading; 



