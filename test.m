function sim_swarm
close all, clc, clear all
load 'cloud1.mat'

% time and time step
t = 0;
dt = 2;
t_max = 1800;
steps = t_max/dt;

swarmSize = 5;
x = zeros(3,swarmSize);
v = 10;
mu = 0.2;
max_dist = 1000;
max_dist_buffer =  max_dist - max_dist*0.1;
message_all = zeros(3,swarmSize);
pollution = 0;

for UAV = 1:swarmSize
    targ{UAV} = [50;50];
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.velCommands = [u{UAV}(1),u{UAV}(2)];
    navMemory{UAV}.navState = 1;
    navMemory{UAV}.velEstimate = [0;0];
end

figure
hold on
%% main simulation loop
for kk=1:steps
    
    t = t + dt;
    
    for UAV=1:swarmSize
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV}, message_all);
        [u{UAV},navMemory{UAV},targ{UAV}] = simNavDecision(y{UAV},u{UAV},navMemory{UAV},targ{UAV},pollution,kk,dt);
        x(:,UAV) = simMove(x(:,UAV),u{UAV},dt);
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t);
        message{UAV} = [y{1}.Position; pollution];
        message_all(:,UAV) =  message{UAV}(:);
    end
    
    cla
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),pollution))
    for UAV = 1 :swarmSize
        if navMemory{UAV}.navState == 1
            col = 'bo';
        elseif navMemory{UAV}.navState == 2
            col = 'go';
        else col = 'ko';
        end
    end
    
        
    
%     scatter(x(1,:),x(2,:),35,'r','filled')
    for UAV=1:swarmSize
        plot(targ{UAV}(1),targ{UAV}(2),'gs')
        plot(x(1,UAV),x(2,UAV),col)
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = sim_GPS(x,UAV,navMemory,targ,message_all)
y.Position = x(1:2,UAV) + 3*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading;
y.DistanceToGoal = sqrt((y.Position(1)-targ(1))^2+(y.Position(2)-targ(2))^2);
danger = [inf;inf];
nearestDist = inf;

for i = 1:size(message_all,2)
thisDist = norm(message_all(1:2,i) - y.Position);
    if (thisDist<nearestDist),
        y.NearestObs = message_all(1:2,i) - y.Position;
        nearestDist = thisDist;
        danger = message_all(1:2,i);
    end 
end

y.HeadingToObs = atan2(danger(1)-y.Position(1),danger(2)-y.Position(2)) - y.Heading;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u,navMemory,targ] = simNavDecision(y,u,navMemory,targ,pollution,iteration,dt)

navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];

switch navMemory.navState
    
       case 1, % spiral move exploration

        u(1) = 10 ;
        u(2) = (1/300)*exp(-0.007*(iteration-1)*dt)/pi*180;
        disp('exploring')
         % if too close, switch to avoidance mode
        if norm(y.NearestObs)<50 && iteration>200
            navMemory.navState = 2;
        elseif pollution > 0.8 && pollution < 1.2
            navMemory.navState = 3;
            targ = y.Position;
        end

        
    case 2, %  obstacle avoidance
        
        
        u(1) = 10 * ((pi/2 - abs(y.HeadingToObs +pi/2)) / (pi/2));
        u(2) = (3*pi/180) * ((y.HeadingToObs + pi/2) / (pi/2));
        disp('avoidance')
        if norm(y.NearestObs)>50
            navMemory.navState = 1;
%             navMemory.lastSwitch = kk; % start timer
%             navMemory.randTarg = 2*(rand(2,1)-0.5);
        end
        
    case 3, % tracking target
        
        if pollution > 0.8 && pollution < 1.2
            targ = y.Position;
        end
        
        u(1) = 10;
        u(2) = 3;
        disp('target reached')
        % if too close, switch to avoidance mode
        if norm(y.NearestObs)<50
            navMemory.navState = 2;
%         elseif pollution < 0.8 || pollution > 1.2
%             navMemory.navState = 1;
        else
            navMemory.navState = 3;
        end
        
end

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
xdot = [u(1)*sin(x(3));...
        u(1)*cos(x(3));...
        u(1)*u(2)];