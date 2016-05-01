function sim_swarm
close all, clc, clear all
load 'cloud1.mat'

% time and time step
t = 0;
dt = 2;
t_max = 1800;
steps = t_max/dt;
xs = []; 

swarmSize = 8;
x = zeros(3,swarmSize);
v = 10;
mu = 0.2;
max_dist = 1000;
max_dist_buffer =  max_dist - max_dist*0.1;
message_all = zeros(3,swarmSize);
pollution = 0;

for UAV = 1:swarmSize
    targ{UAV} = [1000;1000];
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.velCommands = [u{UAV}(1),u{UAV}(2)];
    navMemory{UAV}.navState = 1;
    navMemory{UAV}.velEstimate = [0;0];
end

targ = regularPolygon(swarmSize,max_dist);
figure
hold on
%% main simulation loop
for kk=1:steps
    
    t = t + dt;
     
    % All agents launched?
    for UAV=1:swarmSize
               
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV}, message_all);
        [u{UAV},navMemory{UAV},targ{UAV}] = simNavDecision(y{UAV},u{UAV},navMemory{UAV},targ{UAV},pollution,kk,dt,max_dist_buffer);
        x(:,UAV) = simMove(x(:,UAV),u{UAV},dt);
        xs{UAV}(1:2,kk) = [x(1,UAV),x(2,UAV)];
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t);
        message{UAV} = [y{1}.Position; pollution];
        message_all(:,UAV) =  message{UAV}(:);
    end
    
       cla
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),pollution))
    for UAV = 1 :swarmSize
        if navMemory{UAV}.navState == 1
            col = 'b';
            disp('go to target')
        elseif navMemory{UAV}.navState == 2
            col = 'g';
            disp('explore')
        elseif navMemory{UAV}.navState == 3
            col = 'r';
            disp('explore')
        else 
            col = 'c';
            disp('track target')            
        end
        plot(targ{UAV}(1),targ{UAV}(2),'gs')
        scatter(x(1,UAV),x(2,UAV),35,col,'filled')
        plot( xs{UAV}(1,:),xs{UAV}(2,:),'k-')
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
closest_UAV = [inf;inf];
nearestDist = inf;

for i = 1:size(message_all,2)
thisDist = norm(message_all(1:2,i) - y.Position);
    if (thisDist<nearestDist),
        y.NearestObs = message_all(1:2,i) - y.Position;
        nearestDist = thisDist;
        closest_UAV = message_all(1:2,i);
    end 
end

y.HeadingToObs = atan2(closest_UAV(1)-y.Position(1),closest_UAV(2)-y.Position(2)) - y.Heading;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u,navMemory,targ] = simNavDecision(y,u,navMemory,targ,pollution,iteration,dt,max_dist)

navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];

switch navMemory.navState
    
        
       case 1, % go to target
           
           u(1) = 10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
           u(2) =  (3*pi/180) * (y.HeadingToGoal/(pi/2));
                      
           if pollution > 0.8 && pollution < 1.2
               navMemory.navState = 4; %if close to cloud, track it
            targ = y.Position;
           elseif y.DistanceToGoal < 40;
               navMemory.navState = 3;
           end
         
    case 2, % explore -  make a spiral motion
        
        u(1) = 10 ;
        u(2) = (1/300)*exp(-0.007*(iteration-1)*dt)/pi*180;
        
        if ( norm(y.NearestObs) < 100 || norm(y.Position) > max_dist )
            navMemory.navState = 3; % if too close, switch to avoidance mode
        elseif (iteration - navMemory.lastSwitch) > 100            
            navMemory.navState = 1; % switch back to target tracking after 10s
        elseif pollution > 0.8 && pollution < 1.2 
            navMemory.navState = 4; %if close to cloud, track it
            targ = y.Position;
        end

        
    case 3, %  avoid - avoid other agents and stay inside map
        
        u(1) = 10 * ((pi/2 - abs(y.HeadingToObs + pi)) / (pi/2));
        u(2) = (3*pi/180) * ((y.HeadingToObs + pi) / (pi/2));
        
        if norm(y.NearestObs)>100,
            navMemory.navState = 2; % if far enough away, switch back to explore
            navMemory.lastSwitch = iteration; % start timer
        end
        
%         if y.DistanceToGoal>120
%             navMemory.navState = 1;
%             navMemory.lastSwitch = kk; % start timer
%             navMemory.randTarg = 2*(rand(2,1)-0.5);
%         end
        
    case 4, % tracking target
        
        u(1) = 10;
        u(2) = 3;

       
        
%         if y.DistanceToGoal>20
%             navMemory.navState = 1;
% %         elseif pollution < 0.8 || pollution > 1.2
% %             navMemory.navState = 1;
%         else
%             navMemory.navState = 3;
%         end
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xnew=simMove(x,u,dt)

v = u(1);
mu = u(2);
minv = 10*dt;
maxv = 20*dt;
minmu = -6*dt;
maxmu = 6*dt;
    
% check if within speed limits
    if v < minv
        v = minv;
    elseif v > maxv
        v = maxv;
    end
    
% check if within angular velocity limits
    if mu < minmu
        mu = minmu;
    elseif mu > maxmu
        mu = maxmu;
    end
    
% runge kutta 4th order
k1 = f_continuous(x,v,mu);
k2 = f_continuous(x+k1*dt/2,v,mu);
k3 = f_continuous(x+k2*dt/2,v,mu);
k4 = f_continuous(x+k3*dt,v,mu);

xnew = x+(k1+2*k2+2*k3+k4)*dt/6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot=f_continuous(x,v,mu)
xdot = [v*sin(x(3));...
        v*cos(x(3));...
        v*mu];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
function initial_target = regularPolygon(swarmSize,max_dist)

angle = 2*pi/(swarmSize - 1);
for i=1:swarmSize - 1 
    initial_target{i}= 3/4 * max_dist * [sin(i*angle);cos(i*angle)];
end
initial_target{swarmSize} = [0,0]; 