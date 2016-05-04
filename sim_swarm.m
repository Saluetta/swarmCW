function test
close all
clc
load 'cloud1.mat'
figure
hold on

% time and time step
t = 0;
dt = 2;
t_max = 1800;
steps = t_max/dt;

% system properties
swarmSize = 8;
v = 10;
mu = 0;
max_dist = 1000;
max_dist_buffer =  max_dist - max_dist*0.1; %give a set inner bound
targ = regularPolygon(swarmSize,max_dist); %launch agents to equally spaced targets in the map
channel = initChannel();

% memory allocation
xs = cell(1,swarmSize);
x = zeros(3,swarmSize);
u = cell(swarmSize,1);
y = cell(swarmSize,1);
navMemory = cell(swarmSize,1);
collisions = zeros(steps,swarmSize);
timeToFindTheCloud = zeros(1,swarmSize);
OutsideMap = zeros(steps,swarmSize);
EndDwellers = zeros(1,swarmSize);
for UAV = 1:swarmSize
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.navState = 1;
    navMemory{UAV}.lastSwitch = 0;
    navMemory{UAV}.collisions = 0;
    navMemory{UAV}.TimeToFindTheCloud = 0;
    navMemory{UAV}.OutsideMap = 0;
    navMemory{UAV}.EndDwellers = 0;
end

%% main simulation loop
for kk=1:steps
    
    t = t + dt;
     
    for UAV=1:swarmSize
        
        [msgs,channel] = simReceive(channel);        
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV});
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t);  
        
        [u{UAV},navMemory{UAV},targ{UAV}] = simNavDecision(y{UAV},u{UAV},UAV,...
            navMemory{UAV},targ{UAV},pollution,kk,dt,max_dist_buffer,msgs);      
        
        %~~~~~~~~~~~~~~~~~~~~~~~~~~~~ outputs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        collisions(kk,UAV) = navMemory{UAV}.collisions;
        timeToFindTheCloud(UAV) = navMemory{UAV}.TimeToFindTheCloud;
        OutsideMap(kk,UAV) = navMemory{UAV}.OutsideMap;
        EndDwellers(UAV) = navMemory{UAV}.EndDwellers;
        %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        x(:,UAV) = simMove(x(:,UAV),u{UAV},dt);        
        xs{UAV}(1:2,kk) = [x(1,UAV),x(2,UAV)];
        msg = [y{UAV}.Position; pollution];
        channel = simTransmit(msg, channel);
    end
        
        channel = simChannel(channel);
        cla
        title(sprintf('t=%.1f secs Concentration=%.2f',t,pollution))
    
    for UAV = 1 :swarmSize
        if navMemory{UAV}.navState == 1
            col = 'c'; % cyan = "go to target" state
        elseif navMemory{UAV}.navState == 2
            col = 'b'; % blue = explore state
        elseif navMemory{UAV}.navState == 3
            col = 'r'; % red =  avoidance
        else 
            col = 'g'; % green = cloud is reached!
        end
        scatter(x(1,UAV),x(2,UAV),35,col,'filled')
        plot (xs{UAV}(1,:),xs{UAV}(2,:),':','Color',[0.5 0.5 0.5]) % plot your path
%         plot([x(1,UAV) x(1,UAV)+50*cos(x(3,UAV))],[x(2,UAV) x(2,UAV)+50*sin(x(3,UAV))],col);
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
end

averageUAVcollision = sum(collisions,1);
OutsideMap = sum(OutsideMap,1);
disp('Number of collisions for each agent')
disp(averageUAVcollision)
disp('Time [sec] the agent takes to track the cloud')
disp(timeToFindTheCloud)
disp('Total number of iterations where the agent is outside the map')
disp(OutsideMap)
disp('Agents in the cloud after 30 minutes')
disp(EndDwellers);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = sim_GPS(x,UAV,navMemory,targ)
y.Position = x(1:2,UAV) + 3*randn(2,1); % New estimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading;
y.DistanceToGoal = sqrt((y.Position(1)-targ(1))^2+(y.Position(2)-targ(2))^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u,navMemory,targ] = simNavDecision(y,u,UAV,navMemory,targ,pollution,iteration,dt,max_dist,msgs)

navMemory.lastPos = y.Position;

%% %%%%%%%%%%%%%%%%%%%%%% PROCESS MESSAGES %%%%%%%%%%%%%%%%%%%%%%%%%

msg = zeros(3,size(msgs,1));
if ~isempty(msgs) %the channel is empty during the first iteration
    for jj = 1:size(msgs,1) % for all agents, except yourself
        msg(:,jj) = msgs{jj};
    end
obs = msg(1:2,:); %set everybody's position as an obstacle
msg(:,UAV) = [];
obs(:,UAV) = [];
distances = bsxfun(@minus,y.Position,obs); % compute difference between next position and obstacles
dist_from_zero = norm([y.Position(1) y.Position(2)] - [0 0]); % compute distance from the base

% Save number of collisions with other agents
% find the number of agents which are closer than 2 metres from you
navMemory.collisions = numel(find((sqrt(sum(distances.^2,1)))<2));
    if isempty(navMemory.collisions)
        navMemory.collisions = 0;
    end
% Save the time at which the cloud was found by the agent
    if navMemory.navState == 4 && navMemory.TimeToFindTheCloud == 0
       navMemory.TimeToFindTheCloud = iteration*dt;    
    end
% Save if the agent is outside the bounds
    if dist_from_zero>1000
        navMemory.OutsideMap = 1;
    end
end
% Save who is in the cloud after 30 minutes
if iteration == 900
    if navMemory.navState == 4
        navMemory.EndDwellers = 1;
    end
end            

%% %%%%%%%%%%%%%%%%%%%%%% AVOIDANCE FLAG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
avoidance = 0;
outOfMap = 0;
if iteration > 50 % launching is finished

%calculate a prediction of where you will be next time step with current velocity inputs u
%and current estimated orientation "y.Heading" and position "y.Position"
next_position = simMove([y.Position;y.Heading],u,dt); 
dist_from_zero_next = norm([next_position(1) next_position(2)] - [0 0]); % distance from yourself in the next step and the base
diff_vector_next = bsxfun(@minus,next_position(1:2),obs); % compute difference between next position and obstacles
[min_dist_next,Idx] = min(sqrt(sum(diff_vector_next.^2,1))); % compute minimum distance between you and all obstacles
nearestAgent = obs(:,Idx); %find nearest agent based on its index Idx
HeadingToObs = atan2(nearestAgent(1)-y.Position(1),nearestAgent(2)-y.Position(2)) - y.Heading; % align yourself with obstacle
    if min_dist_next < 80 % if next time step you will be close to map bounds or within 80 from closest obstacle
        avoidance = 1;
    elseif dist_from_zero_next > max_dist
        outOfMap = 1;
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%% SWARMING FLAG %%%%%%%%%%%%%%%%%%%%%%%%%%%
swarming = 0;
rebase = 0;
cloud_dweller = find(msg(3,:)>0.3 & msg(3,:)<1.6); %those who are in the cloud

if ~isempty(cloud_dweller) % if somebody is in the cloud 
    cloudPoints = msg(1:2,cloud_dweller); % your new target is their position
    diff_vector = bsxfun(@minus,y.Position,cloudPoints); % compute difference between next position and obstacles
    [min_dist,Idx] = min(sqrt(sum(diff_vector.^2,1)));
    
    if size(cloud_dweller,2)<6 %if the cloud dwellers are less than 5, starts swarming
        swarming = 1;
        if min_dist>100 % while your distance to the cloud is big, your target is the position of the closest cloud dweller
            targ = cloudPoints(:,Idx);
        elseif min_dist<100 %if you are getting to close to a cloud dweller, your target is shifted on the side
            targ = cloudPoints(:,Idx) + 100*rand(2,1);
        end
    else
        rebase = 1; %if the cloud is too busy, don't go there
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%% FSM %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
switch navMemory.navState    
        
       case 1, % go to target
           
           u(1) = 10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
           u(2) =  (2*pi/180) * (y.HeadingToGoal/(pi/2));
           
           if avoidance
               navMemory.navState = 3;
               navMemory.lastSwitch = iteration; % start timer
           elseif outOfMap
               navMemory.navState = 1;
               targ = [0;0];
           elseif pollution > 0.8 && pollution < 1.2 % in case you are the first one to detect the cloud
               navMemory.navState = 4; %if close to cloud, track it
               targ = y.Position;
           elseif swarming
               navMemory.navState = 1;
               if pollution > 0.8 && pollution < 1.2
                   navMemory.navState = 4; %if close to the cloud, track it
                   targ = y.Position;
               end
           elseif rebase
               navMemory.navState = 2;
               if pollution > 0.8 && pollution < 1.2
                   navMemory.navState = 4; %if close to the cloud, track it
                   targ = y.Position;
               end
           elseif y.DistanceToGoal < 40;
               navMemory.navState = 2; % explore
           end
           
    case 2, % explore -  make a spiral motion
        
        u(1) = 10 ;
        u(2) = ((1/300)*exp(-0.007*(iteration-1)*dt))/pi*180; % makes a spiral growing with iteration loop
        
        if avoidance
            navMemory.navState = 3;
            navMemory.lastSwitch = iteration; % start timer
        elseif outOfMap
               navMemory.navState = 1;
               targ = [0;0];
        elseif pollution > 0.8 && pollution < 1.2 && ~rebase% in case you are the first one to detect the cloud
            navMemory.navState = 4; %if close to cloud, track it
            targ = y.Position;            
        elseif swarming 
            navMemory.navState = 1;
            if pollution > 0.8 && pollution < 1.2
                navMemory.navState = 4; %if close to cloud, track it
                targ = y.Position;
            end
        end
   
    case 3, %  avoid - avoid other agents and stay inside map
        
        u(1) = 10;
        u(2) = (180/pi) * (HeadingToObs + pi); % turn PI in opposite direction of the obstacle
        
        if outOfMap
            navMemory.navState = 1;
            targ = [0;0];
        elseif ~avoidance
            navMemory.navState = 2;
        end
        
    case 4, % tracking target
       % to get inside this state, agent must have pollution > 0.8 && pollution <1.2
       
       if pollution < 0.9 || pollution > 1.1 
            u(1) = 10;
            u(2) = 3;
            if pollution > 1.5 || pollution < 0.6
               navMemory.navState = 2;
            end
        else
            u(1) = 10;
            u(2) = 6;
            targ = y.Position;
        end       
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Move the agent
function xnew=simMove(x,u,dt)

v = u(1);
mu = u(2);
minv = 10;
maxv = 20;
minmu = -6;
maxmu = 6;
    
% check if within speed limits and cap it
if v < minv
    v = minv;
elseif v > maxv
    v = maxv;
end
    
% check if within angular velocity limits and cap it
if mu < minmu
    mu = minmu;
elseif mu > maxmu
    mu = maxmu;
end
        
% Runge Kutta 4th order
k1 = f_continuous(x,v,mu);
k2 = f_continuous(x+k1*dt/2,v,mu);
k3 = f_continuous(x+k2*dt/2,v,mu);
k4 = f_continuous(x+k3*dt,v,mu);

xnew = x+(k1+2*k2+2*k3+k4)*dt/6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamics
function xdot=f_continuous(x,v,mu)
xdot = [v*sin(x(3));...
        v*cos(x(3));...
        v*mu];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spreading of the agents in the map   
function target = regularPolygon(swarmSize,max_dist)

target = cell(1,6);
angle = 2*pi/(swarmSize - 1);

for i=1:swarmSize - 1 
    target{i}= 3/4 * max_dist * [sin(i*angle);cos(i*angle)];
end
target{swarmSize} = [0,0]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Communications
function channel = initChannel()
    % initialize comms channel model
    channel.curMsgs = {};
    channel.newMsgs = {};

function [rxMsgs,channel] = simReceive(channel)
    % simulate receiving messages
    % simple broadcast model - just get everything
    rxMsgs = channel.curMsgs;
    
function channel = simTransmit(txMsgs,channel)
    % simulate transmitting a message
    % store it for next step
    channel.newMsgs = [channel.newMsgs; txMsgs];

function channel = simChannel(channel)
    % simple - everyone gets everything
    channel.curMsgs = channel.newMsgs;
    channel.newMsgs = {};