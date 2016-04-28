function sim_swarm_salua
clc, close all, clear all
load 'cloud1.mat'

% TODO:
% convergence of uavs around contour
% set boundaries on acceleration
% condition on "equally" spaced uavs
% prevent collisions of uavs and keep them inside map

% time and time step
t = 0;
dt = 3.6;

swarmSize = 5;
x = zeros(3,swarmSize);
% pollution = 0; % no pollution at the beginning
% control inputs
v = 10; mu = 0;

% initialise
for UAV = 1:swarmSize
    navMemory{UAV}.lastPos = [0,0];
    navMemory{UAV}.navState = 1;
    u{UAV} = [v mu]; % last recorded position
%     navMemory.velEstimate = [v*sin(0);v*cos(0);v*mu]; % estimate of own velocity
    navMemory{UAV}.velCommands = [u{UAV}(1),u{UAV}(2)];
    
    targ{UAV} = [500,0];
%     targ{UAV} = randi([-500,500],1,2); %random initial target within 1 kilometre
%     message(UAV,:) = [navMemory{UAV}.lastPos, pollution];
end

figure
hold on
%% main simulation loop
for kk=1:1800 %endurance of each UAV is 30 minutes = 1800 seconds    
    t = t + dt;
    
    for UAV=1:swarmSize 
%         if kk>40
%             targ{UAV} = new_targ;
%         end
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV}); % simulate own position measurement via GPS       
        [u{UAV},navMemory{UAV}] = simNavDecision(y{UAV},u{UAV},navMemory{UAV}); % UAV makes decision on acceleration
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t) % take measurement of pollution
%         message(UAV,:) = [x(1,UAV),x(2,UAV), pollution];
        if pollution > 0.8 && pollution < 1.2
            targ{UAV} = y{agent}.Position;
        end
    end
% %     while kk<100
%     getting_closer = min(abs(1 - pollution));
%     close_UAV = find(getting_closer == abs(1 - pollution));
% %     end
    
%     new_targ = [x(1,close_UAV),x(2,close_UAV)];
%     new_pollution = min(pollution)
  
     %  ~~~~~~~~~~~~~~~~~~~~~~~~~~~ plot ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cla
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),pollution))
    plot(x(1,:),x(2,:),'bo')
    for UAV=1:swarmSize
        plot(targ{UAV}(1),targ{UAV}(2),'gs')
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
    %  ~~~~~~~~~~~~~~~~~~~~~~~~~~~ move ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for UAV=1:swarmSize
        % execute decision
        x(:,UAV) = simMove(x(:,UAV),u{UAV},navMemory{UAV},dt);
        navMemory{UAV}.lastPos = [x(1,UAV),x(2,UAV)];
    end 
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
u(1) = 10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
u(2) =  (3*pi/180) * (y.HeadingToGoal/(pi/2));

% control - accelerate to that velocity
%u = 0.3*(targVel - navMemory.velEstimate);


% % % % simulate agent deciding own acceleration
% % % 
% % % % first get updated velocity estimate
% % % % start with direct differencing from last time
% % % noisyVel = (y.ownPosition - navMemory.lastPos)/0.1;
% % % % and filter to remove some of the noise
% % % navMemory.velEstimate = navMemory.velEstimate + ...
% % %                  0.2*(noisyVel - navMemory.velEstimate);
% % % % reset last position store with new measurement
% % % navMemory.lastPos = y.ownPosition;
% % % 
% % % % guidance - constant speed towards target
% % % targVel = y.targetVector*0.1/norm(y.targetVector);
% % % 
% % % % control - accelerate to that velocity
% % % u = 0.3*(targVel - navMemory.velEstimate);





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
%xdot = [v*sin(theta);v*cos(theta);v*mu]
xdot = [u(1)*sin(x(3));...
        u(1)*cos(x(3));...
        u(1)*u(2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = sim_GPS(x,agent,navMemory,targ)
% simulate measurement in three parts for each agent
% first is position, plus noise
y.Position = x(1:2,agent) + 3*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New estimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading;
