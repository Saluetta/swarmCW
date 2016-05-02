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
max_dist_buffer =  max_dist - max_dist*0.1;
targ = regularPolygon(swarmSize,max_dist);
channel = initChannel();

% memory allocation
xs = cell(1,swarmSize);
x = zeros(3,swarmSize);
u = cell(swarmSize,1);
y = cell(swarmSize,1);
navMemory = cell(swarmSize,1);

for UAV = 1:swarmSize
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.navState = 1;
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
            col = 'c';
        elseif navMemory{UAV}.navState == 2
            col = 'b';
        elseif navMemory{UAV}.navState == 3
            col = 'r';
        else 
            col = 'g';
        end
        scatter(x(1,UAV),x(2,UAV),35,col,'filled')
        plot (xs{UAV}(1,:),xs{UAV}(2,:),':','Color',[0.5 0.5 0.5])
%         plot([x(1,UAV) x(1,UAV)+50*cos(x(3,UAV))],[x(2,UAV) x(2,UAV)+50*sin(x(3,UAV))],col);
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
end

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
end

%% %%%%%%%%%%%%%%%%%%%%%% AVOIDANCE FLAG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
avoidance = 0;
if iteration > 50 % launching is finished   
next_position = simMove([y.Position;y.Heading],u,dt); %calculate position with this state's velocity inputs u
dist_from_zero_next = norm([next_position(1) next_position(2)] - [0 0]); % distance from the base
diff_vector_next = bsxfun(@minus,next_position(1:2),obs); % compute difference between next position and obstacles
[min_dist_next,Idx] = min(sqrt(sum(diff_vector_next.^2,1))); % compute minimum distance between you and all obstacles
nearestAgent = obs(:,Idx); %find nearest agent based on its Idx
HeadingToObs = atan2(nearestAgent(1)-y.Position(1),nearestAgent(2)-y.Position(2)) - y.Heading; % align yourself with obstacle
    if dist_from_zero_next > max_dist || min_dist_next < 80 % if close to map bounds or within 80 from closest obstacle
       avoidance = 1;
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%% SWARMING FLAG %%%%%%%%%%%%%%%%%%%%%%%%%%%
swarming = 0;

cloud_dweller = find(msg(3,:)>0.5); %those who are in the cloud
if ~isempty(cloud_dweller) && size(cloud_dweller,2)<5 % if somebody is in the cloud
   targ = msg(1:2,cloud_dweller); % target is their position
   swarming =1;
end

switch navMemory.navState    
        
       case 1, % go to target
           
           u(1) = 10 ;
           u(2) = y.HeadingToGoal*pi/180;
           
           if avoidance
               navMemory.navState = 3;
           elseif swarming
               navMemory.navState = 1;
           elseif y.DistanceToGoal < 40;
               navMemory.navState = 2; % explore
           elseif pollution > 0.8 && pollution < 1.2
               navMemory.navState = 4; %if close to the cloud, track it
               targ = y.Position;
           end
           
    case 2, % explore -  make a spiral motion
        
        u(1) = 10 ;
        u(2) = ((1/300)*exp(-0.007*(iteration-1)*1.5*dt))/pi*180; % makes a spiral growing with iteration loop
        
        if avoidance
            navMemory.navState = 3;
            avoidance.timer = avoidance.timer + iteration;
        elseif swarming 
            navMemory.navState = 1;
        elseif pollution > 0.8 && pollution < 1.2
            navMemory.navState = 4; %if close to cloud, track it
            targ = y.Position;
        end
   
    case 3, %  avoid - avoid other agents and stay inside map
        
        u(1) = 10;
        u(2) = (180/pi) * (HeadingToObs + pi); % turn PI in opposite direction of the obstacle
        
        if ~avoidance
            navMemory.navState = 2;
        end
        
    case 4, % tracking target
                
        if pollution < 0.9 || pollution > 1.1
            u(1) = 3;
            u(2) = 3;
            if pollution>2.5
               navMemory.navState = 2;
            end
        else
            u(1) = 1;
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