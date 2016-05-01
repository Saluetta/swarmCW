function mysim_nav_swarm_comms

% initial dynamic state [pos;vel] in 2D
% each column is one agent
% random initial position, stopped
Nagents = 30;
x = [3*2*(rand(2,Nagents)-0.5)-3;
    zeros(2,Nagents)];

% initial channel simulation for comms
channel = initChannel();

% initial decision maker memory
% same for each agent
for aa=1:Nagents,
    % last recorded position
    navMemory{aa}.lastPos = [-4;-4];
    % estimate of own velocity
    navMemory{aa}.velEstimate = [0;0];
    % initial operating state
    navMemory{aa}.navState = 1;
    % 1 = drive to target (start)
    % 2 = avoid obstacle
    % 3 = random escape
    % 4 = stop at target
end

% time step
dt = 0.1;

% target position
targ = [4.5;4.5];

% time loop
for kk=1:20000,    
    % loop through decision making
    for aa=1:Nagents,        
        
        % simulate own position measurement
        y{aa} = simMeas(x,aa,targ);
        
        % simulate received message
        [rxMsgs{aa},channel] = simReceive(aa,channel);
        
        % agent makes decision on acceleration and sends message
        [u{aa},navMemory{aa},txMsgs{aa}] = simNavDecision(y{aa},kk,...
                                               navMemory{aa},rxMsgs{aa});
        
        % simulate transmission
        channel = simTransmit(txMsgs{aa},aa,channel);
    end
    
    % simulate the comms
    channel = simChannel(channel,x);
    
    % plot stuff
    plot(x(1,:),x(2,:),'bo', ...
        [x(1,:); x(1,:)+5*x(3,:)],[x(2,:); x(2,:)+5*x(4,:)],'b-',...
        targ(1),targ(2),'gs')    
    axis equal
    axis([-7 7 -7 7])
    pause(dt*0.1) % just to get graphics to redraw
    
    % loop through simulating
    for aa=1:Nagents,        
        % execute decision
        x(:,aa) = simMove(x(:,aa),u{aa},dt);        
    end    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = simMeas(x,aa,targ)
% simulate measurement for agent aa

% first is position, plus noise
y.ownPosition = x(1:2,aa) + 0.01*randn(2,1);

% also measuring relative target position
y.targetVector = targ - x(1:2,aa) + 0.01*randn(2,1);

% obstacles are other agents
obs = x(1:2,[1:(aa-1) (aa+1):end]);
% search for nearest obstacle - start with inf
nearestDist = inf;
% now loop through others
for jj=1:size(obs,2),
    % calc range
    thisDist = norm(x(1:2,aa) - obs(1:2,jj));
    if (thisDist<nearestDist),
        % update incumbent
        y.nearestObs = obs(1:2,jj) - x(1:2,aa);
        nearestDist = thisDist;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u, navMemory, txMsg] = simNavDecision(y,kk,navMemory, rxMsgs)
% simulate agent deciding own acceleration

% first get updated velocity estimate
% start with direct differencing from last time
noisyVel = (y.ownPosition - navMemory.lastPos)/0.1;
% and filter to remove some of the noise
navMemory.velEstimate = navMemory.velEstimate + ...
    0.2*(noisyVel - navMemory.velEstimate);
% reset last position store with new measurement
navMemory.lastPos = y.ownPosition;

% default transmit message
txMsg = [];

% finite state machine for velocity
switch navMemory.navState
    
    case 1, % state = 1 - drive to target
        
        % guidance - constant speed towards target
        targVel = y.targetVector*0.1/norm(y.targetVector);        
        
        if norm(y.nearestObs)<0.5,
            % if too close, switch to avoidance mode
            navMemory.navState = 2;
        elseif norm(y.targetVector)<0.5,
            % if close to target, switch to stop mode
            navMemory.navState = 4;
        else
            % test comms and see if I'm close to anyone else stopped
            for mm=1:numel(rxMsgs),
                if norm(y.ownPosition - rxMsgs{mm})<0.7,
                    navMemory.navState = 4;
                    break
                end
            end
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
        
        % transmit my position
        txMsg = y.ownPosition;
        
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function channel = initChannel()
% initialize comms channel model
channel.curMsgs = {};
channel.newMsgs = {};

function [rxMsgs,channel] = simReceive(aa,channel)
% simulate receiving messages
% simple broadcast model - just get everything
rxMsgs = channel.curMsgs;

function channel = simTransmit(txMsgs,aa,channel)
% simulate transmitting a message
% store it for next step
channel.newMsgs = [channel.newMsgs txMsgs];

function channel = simChannel(channel,x)
% simple - everyone gets everything
channel.curMsgs = channel.newMsgs;
channel.newMsgs = {};