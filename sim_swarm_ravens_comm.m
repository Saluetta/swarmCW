function sim_swarm_ravens_comm
%
% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 2;%%3.6

Num_agents = 7;
Dist_max = 1000;
x = zeros(3,Num_agents); % Initial State {only position}

v = 10;
mu = 0;
% initial channel simulation for comms
channel = initChannel();


for agent = 1:Num_agents,
    % last recorded position
    navMemory{agent}.lastPos = [0;0];
    navMemory{agent}.navState = 1;% initial decision maker memory
    navMemory{agent}.avoidanceTime = randi([5*dt 10*dt],1); %Each UAV has its own evation time between [2*dt,10dt]
    navMemory{agent}.avoidanceTimeCounter = 0;
    navMemory{agent}.visitCloud = 0;
    u{agent} = [v;mu]; % Initial input
    navMemory{agent}.cloudLocation = [];
    
end


[border_targ,x] = spreading(Num_agents,Dist_max,x);
curr_targ = border_targ; 

% v units -> m/s || mu units -> rad/s

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000
    
    % time
    t = t + dt;

    for agent=1:Num_agents
             
        % simulate own position measurement
        y{agent} = sim_GPS(x,agent,navMemory{agent},curr_targ{agent});
        
        % take measurement
        p = cloudsamp(cloud,x(1,agent),x(2,agent),t);
        
        % simulate received message
        [rxMsgs{agent},channel] = simReceive(agent,channel);
        
        % agent makes decision on acceleration
        [u{agent},navMemory{agent},txMsgs{agent},curr_targ{agent}] = simNavDecision(y{agent},u{agent},navMemory{agent},rxMsgs{agent},curr_targ{agent},border_targ{agent},p,kk); 
        
        %Debugging  Record positions
        xs{agent}(1:2,kk) = [x(1,agent),x(2,agent)];
        % simulate transmission
        channel = simTransmit(txMsgs{agent},agent,channel);
    end
    
    channel = simChannel(channel,x);
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),p))
     
    for agent=1:Num_agents
         if navMemory{agent}.navState == 1
        clr = 'bo';
         elseif navMemory{agent}.navState == 2
        clr = 'go';
         else
        clr = 'ro';
         end
        plot(x(1,agent),x(2,agent),clr)
        plot(curr_targ{agent}(1),curr_targ{agent}(2),'gs')   
        %plot( xs{agent}(1,:),xs{agent}(2,:),'c-')
    end
    
    % plot the cloud contours
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    % pause ensures that the plots update
    pause(0.1)
    
     for agent=1:Num_agents,        
        % execute decision
        x(:,agent) = simMove(x(:,agent),u{agent},navMemory{agent},dt);
    end   
    
end


function [u, navMemory, txMsg, targ] = simNavDecision(y,u,navMemory,rxMsgs,targ,border_targ,p,iter)
% simulate agent deciding own acceleration

navMemory.lastPos = y.Position;
closer_Raven = [];
closest_Raven = inf;
distToCloud = inf;

% Estimate distance to others UAV after launching
if iter > 20
    for i = 1:size(rxMsgs,2)
        closer_Raven = [closer_Raven norm(rxMsgs{i}(1:2)-y.Position)];
        if rxMsgs{i}(3) ~= 0 && rxMsgs{i}(4) ~= 0
            navMemory.cloudLocation = [navMemory.cloudLocation,[rxMsgs{i}(3);rxMsgs{i}(4)]];
        end
    end
    closer_Raven = sort(closer_Raven);
    closest_Raven = closer_Raven(2);
    navMemory.cloudLocation;
end

% If a smoke cloud has been detected, get the contours and distance
% to the shortest contour, if the distance is smaller than a threshold
% {50}, that point will be the new target

if size(navMemory.cloudLocation,2)>3
    
    navMemory.cloudLocation = getCloudContours(navMemory);
    distToCloud = distanceToCloud(navMemory);
    
    if distToCloud < 100
        targ = contourTarget(targ,navMemory)+(-50 + (100).*rand(2,1));
    end
     
end

% If a there's an UAV closer than a threshold {80} go to state 4 {Avoidance}
if (closest_Raven < 80 || navMemory.avoidanceTimeCounter ~= 0) && iter > 20
    navMemory.navState = 4;
end

% default transmit message {Position and Position of ppm ~=1 }
txMsg = [y.Position;[0;0]];

switch navMemory.navState
    
    case 1, % Exploring
       
       % Input proportionally to the heading to target 
       u(1) = 10 * ((pi/2 - abs(y.HeadingToTarget))/(pi/2));
       u(2) =  (2*pi/180) * (y.HeadingToTarget/(pi/2));
       
       u(1) = sat(u(1),10,20);
       u(2) = sat(u(2),-6*pi/180,6*pi/180);    
        
        navMemory.navState = 1;
        
        if p > 0.8 && p < 1.2
            navMemory.navState = 2;
        end
        
%         if size(navMemory.cloudLocation,2)>3
%             distToCloud = distanceToCloud(navMemory);
%             if distToCloud < 50
%                 targ = contourTarget(targ,navMemory);
%             end
%         end
        
        if navMemory.visitCloud == 1
            targ = contourTarget(targ,navMemory);
        else
            if y.DistanceToTarget < 50 && y.DistanceToHome > 800
               targ = rot(targ, -pi + 2*pi*rand(1));
               %targ = rot(border_targ,pi/4 * (-1 + (2).*rand(1)) );
            elseif y.DistanceToTarget < 50 && y.DistanceToHome < 100
               targ = border_targ;   
            elseif y.DistanceToHome > 1000
                navMemory.navState = 3;
            end
        end

        txMsg = [y.Position;[0;0]];
        
    case 2, % state = 2 - Track the cloud
        
         navMemory.visitCloud = 1;
        
         if p > 0.8 && p < 1.2
                targ = y.Position;
                navMemory.navState = 2;
                    
                u(1) = 10;
                u(2) =  (6*pi/180);
         else 
                targ = y.Position;
                    
                u(1) = 10 ;
                u(2) =  (1*pi/180) * (y.HeadingToTarget/(pi/4));
                navMemory.navState = 1;
         end
         
           
        u(1) = 10 ;
        u(2) =  (6*pi/180) ;%* (y.HeadingToTarget/(pi/4));
        
        u(1) = sat(u(1),10,20);
        u(2) = sat(u(2),-6*pi/180,6*pi/180);
        
        txMsg = [y.Position;y.Position];
    
         
         
    case 3, % state = 3 -  Go home
          
        if y.DistanceToHome > 950
           targ = [0;0];
           navMemory.navState = 1;
        else
           targ = border_targ;
           navMemory.navState = 1;
        end
        
    case 4, % avoid colision
        
        navMemory.avoidanceTimeCounter = navMemory.avoidanceTimeCounter + 2; %dt
        
        if navMemory.avoidanceTimeCounter > navMemory.avoidanceTime
             navMemory.navState = 1;
             navMemory.avoidanceTimeCounter = 0;
             if navMemory.visitCloud == 1
                targ = contourTarget(targ,navMemory) + (-50 + (100).*rand(2,1));
             else
                targ = targ+(-50 + (100).*rand(2,1));    
             end
        end
        
        if navMemory.avoidanceTimeCounter <= 6%navMemory.avoidanceTime/4
        u(1) =  20;
        u(2) = 6*pi/180;
        else
        u(1) =  20;
        u(2) = 0;            
        end
        
        
end

%%%%%%%%%%%%%%%%%

function [target,x]=spreading(Num_agents,Dist_max,x)
% Targets equally spaced around a circle with radious = Dist_max-100 
theta = 2*pi/Num_agents;
for i=1:Num_agents 
target{i}=  (Dist_max-100) * [sin(i*theta);cos(i*theta)]; 
x(3,i) = getAngle(i*theta);
end

function [target]=contourTarget(target,navMemory)

if size(navMemory.cloudLocation,2)>3
    contourPoints = [];
    index = 0;
    for i = 1:size(navMemory.cloudLocation,2)
        contourPoints = [contourPoints norm(navMemory.cloudLocation(:,i)-navMemory.lastPos)];
    end
    [~,index] = min(contourPoints);
    target = [navMemory.cloudLocation(1,index);navMemory.cloudLocation(2,index)];
end

function [contourPoints]=getCloudContours(navMemory)

if size(navMemory.cloudLocation,2)>3
    contourPoints = [];
    %for i = 1:size(navMemory.cloudLocation,2)
    index = convhull(navMemory.cloudLocation(1,:),navMemory.cloudLocation(2,:));
    for i = 1:size(index)
        contourPoints = [contourPoints [navMemory.cloudLocation(1,index);navMemory.cloudLocation(2,index)]];
    end
    contourPoints;
    %[~,index] = max(contourPoints);
    %target = [navMemory.cloudLocation(1,index);navMemory.cloudLocation(2,index)];
end


function dist = distanceToCloud(navMemory)

if size(navMemory.cloudLocation,2)>3
    distances = [];
    for i = 1:size(navMemory.cloudLocation,2)
        distances = [distances norm( navMemory.lastPos - navMemory.cloudLocation(i))];
    end
    dist = min(distances);
end
   

function angle=getAngle(angle)

if angle > pi
    angle = angle - 2*pi;
elseif angle < -pi
    angle = angle + 2*pi; 
end


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
        u(1)*u(2);];

function y = sim_GPS(x,aa,navMemory,targ)
                    %(x,aa,navMemory,targ)
% simulate measurement for agent aa
% measurement in three parts

% first is position, plus noise
y.Position = x(1:2,aa) + 3*randn(2,1); % New stimated position plus noise
y.DistanceToTarget = sqrt((y.Position(1)-targ(1))^2+(y.Position(2)-targ(2))^2);
y.DistanceToHome = sqrt((y.Position(1)-0)^2+(y.Position(2)-0)^2);
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToTarget = getAngle(atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading);
 

function val = sat(val,a,b)

if val < a
    val = a;
elseif val > b
    val = b;
end

function coord = rot (coord, theta)

coord = [cos(theta) -sin(theta); sin(theta) cos(theta)] * coord;


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

