function sim_swarm_ravens
load 'cloud2.mat'
figure
hold on

% time and time step
t = 0;
dt = 3.6;

Num_agents = 10;
x = zeros(3,Num_agents);

% control inputs
v = 10;
mu = 0;

for agent = 1:Num_agents
    u{agent} = [v mu];
    navMemory{agent}.lastPos = [0,0]; % last recorded position
%     navMemory.velEstimate = [v*sin(0);v*cos(0);v*mu]; % estimate of own velocity
	navMemory{agent}.velCommands = [u{agent}(1),u{agent}(2)];
    targ{agent} = [500;0];
    
end

%% main simulation loop
for kk=1:1000   
    
    t = t + dt;   
    
    for agent=1:Num_agents
             
        % simulate own position measurement
        % y = sim_GPS(x,navMemory,targ);
        y{agent} = sim_GPS(x,agent,navMemory{agent},targ{agent});
        
        % agent makes decision on acceleration
        [u{agent},navMemory{agent}] = simNavDecision(y{agent},u{agent},navMemory{agent});        
    end
    
    % agent makes decision on acceleration
    %u = [1.2;1.2;6*pi/180];%simNavDecision(y,kk,navMemory);
    %x = simMove(x,u,navMemory,dt);
   
    for agent=1:Num_agents
        p = cloudsamp(cloud,x(1,agent),x(2,agent),t); % take measurement

        if p > 0.8 && p < 1.2
            targ{agent} = y{agent}.Position;
        end
    
    end
    
   %% PLOT
    cla 
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),p))
    plot(x(1,:),x(2,:),'bo') % plot robot location 
    for agent=1:Num_agents
        plot(targ{agent}(1),targ{agent}(2),'gs')    
    end
    
    % plot the cloud contours
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1) % pause ensures that the plots update
    
     for agent=1:Num_agents,        
        % execute decision
        x(:,agent) = simMove(x(:,agent),u{agent},navMemory{agent},dt);        
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
        u(1)*u(2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function y = sim_GPS(x,agent,navMemory,targ)                   
% simulate measurement for agent
% measurement in three parts

% first is position, plus noise
y.Position = x(1:2,agent) + 3*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New estimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading; 



