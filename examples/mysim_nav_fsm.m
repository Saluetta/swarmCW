function mysim_nav_fsm

% initial dynamic state [pos;vel] in 2D
x = [-4;-4;
    zeros(2,1)];

% initial decision maker memory
% last recorded position
navMemory.lastPos = [-4;-4];
% estimate of own velocity
navMemory.velEstimate = [0;0];
% initial operating state
navMemory.navState = 1;
% 1 = drive to target (start)
% 2 = avoid obstacle
% 3 = random escape
% 4 = stop at target

% time step
dt = 0.1;

% target position
targ = [4.5;4.5];

% obstacles
obs = 4*2*(rand(2,30)-0.5);

% time loop
for kk=1:20000,
    
    % simulate own position measurement
    y = simMeas(x,targ,obs);
    
    % agent makes decision on acceleration
    [u,navMemory] = simNavDecision(y,kk,navMemory);
    
    % store
    xs(1:4,kk) = x;
    
    % plot stuff
    plot(x(1),x(2),'bo', ...
        xs(1,:),xs(2,:),'c-',...
        x(1)+5*[0 x(3)],x(2)+5*[0 x(4)],'b-',...
        obs(1,:), obs(2,:), 'rx', ...
        targ(1),targ(2),'gs')
    axis equal
    axis([-5 5 -5 5])
    title(sprintf('State = %i',navMemory.navState))
    pause(dt*0.1) % just to get graphics to redraw    
    
    % execute decision
    x = simMove(x,u,dt);
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = simMeas(x,targ,obs)
% simulate measurement for agent aa
% measurement in three parts

% first is position, plus noise
y.ownPosition = x(1:2) + 0.01*randn(2,1);

% also measuring relative target position
y.targetVector = targ - x(1:2) + 0.01*randn(2,1);

% search for nearest obstacle - start with inf
nearestDist = inf;
% now loop through others
for jj=1:size(obs,2),
    % calc range
    thisDist = norm(x(1:2) - obs(1:2,jj));
    if (thisDist<nearestDist),
        % update incumbent
        y.nearestObs = obs(1:2,jj) - x(1:2);
        nearestDist = thisDist;
    end    
end

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

% finite state machine for velocity
switch navMemory.navState
    
    case 1, % state = 1 - drive to target
        
        % guidance - constant speed towards target
        targVel = y.targetVector*0.1/norm(y.targetVector);
        
        % if too close, switch to avoidance mode
        if norm(y.nearestObs)<0.5,
            navMemory.navState = 2;
        elseif norm(y.targetVector)<0.5,
            navMemory.navState = 4;
        end
                
    case 2, % state = 2 - obstacle back away
        
        % guidance - constant speed away from nearest obstacle
        targVel = y.nearestObs*(-0.1)/norm(y.nearestObs);
        
        % if far enough away, switch back to target
        if norm(y.nearestObs)>0.75,
            navMemory.navState = 3;
            navMemory.lastSwitch = kk; % start timer
            navMemory.randTarg = 2*(rand(2,1)-0.5);
        end
        
    case 3, % state = 3 - random move for a bit
        
        % guidance - constant speed away from nearest obstacle
        targVel = 0.1*navMemory.randTarg/norm(navMemory.randTarg);
        
        % if too close, switch to avoidance mode
        if norm(y.nearestObs)<0.5,
            navMemory.navState = 2;
        elseif (kk-navMemory.lastSwitch)>100,
            % switch back to target tracking after 10s
            navMemory.navState = 1;
        end
        
    case 4, % state = 4 - got there
        
        % guidance - stop
        targVel = [0;0];
        
end

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
