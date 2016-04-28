function sim_swarm
close all, clc, clear all
load 'cloud1.mat'

% time and time step
t = 0;
dt = 3.6;
t_max = 1800;
steps = t_max/dt;

pollution = 0;
swarmSize = 5;
x = zeros(3,swarmSize);
v = 10;
mu = 0.2;
max_dist = 1000;
max_dist_buffer =  max_dist - max_dist*0.1;
targ_x = linspace(0,max_dist_buffer,swarmSize);
targ_y = randi([-max_dist_buffer,max_dist_buffer],1,swarmSize);

for UAV = 1:swarmSize
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.velCommands = [u{UAV}(1),u{UAV}(2)];
    navMemory{UAV}.navState = 3;
    navMemory{UAV}.velEstimate = [0;0];
    targ{UAV} = [targ_x(UAV);targ_y(UAV)];
end

figure
hold on
%% main simulation loop
for kk=1:steps

    t = t + dt;
    
    for UAV=1:swarmSize        
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV});
        [u{UAV},navMemory{UAV}] = simNavDecision(y{UAV},u{UAV},navMemory{UAV},kk,UAV,pollution);
        x(:,UAV) = simMove(x(:,UAV),u{UAV},dt);
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t);        
        if pollution > 0.9 && pollution < 1.1
            targ{UAV} = y{UAV}.Position;
        end        
    end
    
    cla
%     title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),pollution))
    
    scatter(x(1,:),x(2,:),35,'r','filled')
    for UAV=1:swarmSize
        plot(targ{UAV}(1),targ{UAV}(2),'gs')
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = sim_GPS(x,UAV,navMemory,targ)
y.Position = x(1:2,UAV) + 3*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading;
y.targetVector = targ - x(1:2,UAV) + 0.01*randn(2,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u_new, navMemory] = simNavDecision(y,u,navMemory,kk,UAV,pollution)

% noisyVel = (y.Position - navMemory.lastPos)/0.1;
% navMemory.velEstimate = navMemory.velEstimate + 0.2*(noisyVel - navMemory.velEstimate);
navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];


switch navMemory.navState
    
    case 1, % state = 1 - drive to target        
       
        targVel = [10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));(3*pi/180) * (y.HeadingToGoal/(pi/2))];
        
        if (y.targetVector(1)<10 || y.targetVector(2)<10) && (pollution<0.9 || pollution>2)  % if too close, switch to avoidance mode
            navMemory.navState = 3;
        end

% if norm(y.nearestObs)<0.5,
%             navMemory.navState = 2;
%         elseif norm(y.targetVector)<0.5,
%             navMemory.navState = 4;
%         end
                
    case 2, % state = 2 - obstacle back away
        
        % guidance - constant speed away from nearest obstacle
%         targVel = y.nearestObs*(-0.1)/norm(y.nearestObs);
%         
%         % if far enough away, switch back to target
%         if norm(y.nearestObs)>0.75,
%             navMemory.navState = 3;
%             navMemory.lastSwitch = kk; % start timer
%             navMemory.randTarg = 2*(rand(2,1)-0.5);
%         end
        
    case 3, % state = 3 - random move for a bit
            if UAV == 1
            targVel = [10;0.001];
            elseif UAV == 2
            targVel = [10;0.007];
            elseif UAV == 3
            targVel = [10;0.01];
            elseif UAV == 4
            targVel = [10;0.05];
            elseif UAV == 5
            targVel = [10;0.1];
            end
            % guidance - constant speed away from nearest obstacle

         if pollution > 0.9 && pollution < 1.1          
               navMemory.navState = 4;
         end
        
%         % if too close, switch to avoidance mode
%         if norm(y.nearestObs)<0.5,
%             navMemory.navState = 2;
%         elseif (kk-navMemory.lastSwitch)>100,
%             % switch back to target tracking after 10s
%             navMemory.navState = 1;
%         end
        
    case 4, % state = 4 - got there
        targVel = [0.5;0.5];
        
end
u_new = 1*(targVel - navMemory.velEstimate);

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