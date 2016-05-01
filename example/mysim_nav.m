function mysim_nav

% initial dynamic state [pos;vel] in 2D
x = [-4;-4;
    zeros(2,1)];

% initial decision maker memory
% last recorded position
navMemory.lastPos = [-4;-4]; 
% estimate of own velocity
navMemory.velEstimate = [0;0]; 

% time step
dt = 0.1;

% target position
targ = [4;4];

% time loop
for kk=1:20000,
    
    % simulate own position measurement
    y = simMeas(x,targ);
    
    % agent makes decision on acceleration
    [u,navMemory] = simNavDecision(y,kk,navMemory);
    
    % plot stuff
    plot(x(1),x(2),'bo', ...
        x(1)+5*[0 x(3)],x(2)+5*[0 x(4)],'b-',...
        targ(1),targ(2),'gs')
    axis equal
    axis([-5 5 -5 5])
    %grid on
    pause(dt*0.1) % just to get graphics to redraw
    
    % execute decision
    x = simMove(x,u,dt);
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = simMeas(x,targ)
% simulate measurement for agent aa
% measurement in three parts

% first is position, plus noise
y.ownPosition = x(1:2) + 0.01*randn(2,1);

% also measuring relative target position
y.targetVector = targ - x(1:2) + 0.01*randn(2,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u, navMemory] = simNavDecision(y,kk,navMemory)
% simulate agent deciding own acceleration

% first get updated velocity estimate
% start with direct differencing from last time
noisyVel = (y.ownPosition - navMemory.lastPos)/0.1;
% and filter to remove some of the noise
navMemory.velEstimate = navMemory.velEstimate + ...
                 0.2*(noisyVel - navMemory.velEstimate);
% reset last position store with new measurement
navMemory.lastPos = y.ownPosition;

% guidance - constant speed towards target
targVel = y.targetVector*0.1/norm(y.targetVector);

% control - accelerate to that velocity
u = 0.3*(targVel - navMemory.velEstimate);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xnew=simMove(x,u,dt)
% runge kutta 4th order
k1 = f_continuous(x,u);
k2 = f_continuous(x+k1*dt/2,u);
k3 = f_continuous(x+k2*dt/2,u);
k4 = f_continuous(x+k3*dt,u);
xnew = x+(k1+2*k2+2*k3+k4)*dt/6;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot=f_continuous(x,u)
% simple point mass: x is [2D pos; 2D vel] and u is [2D acc]
% so xdot is just a rearrange
xdot = [x(3:4); u];
