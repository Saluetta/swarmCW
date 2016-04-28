function sim_swarm
close all, clc, clear all
load 'cloud1.mat'

% time and time step
t = 0;
dt = 3.6;
t_max = 1800;
steps = t_max/dt;

swarmSize = 5;
x = zeros(3,swarmSize);
v = 10;
mu = 0.2;
max_dist = 1000;
max_dist_buffer =  max_dist - max_dist*0.1;
% targ_x = linspace(0,max_dist_buffer,swarmSize);
% targ_y = randi([-max_dist_buffer,max_dist_buffer],1,swarmSize);
pollution = 0;
turns = linspace(0,10*pi,100);
targ_x = 15*turns.*cos(turns);
targ_y = 15*turns.*sin(turns);
targ_spiral = [targ_x;targ_y];
for UAV = 1:swarmSize
    u{UAV} = [v;mu];
    navMemory{UAV}.lastPos = [0;0];
    navMemory{UAV}.velCommands = [u{UAV}(1),u{UAV}(2)];
    navMemory{UAV}.navState = 1;
    navMemory{UAV}.velEstimate = [0;0];
end

figure
hold on
i=1;
%% main simulation loop
for kk=1:steps
    
    t = t + dt;
    
    if pollution<0.8 || pollution>1.2        
        target = [targ_spiral(:,i)];
        i=i+1;
        if i == length(targ_spiral) 
            i = 1;
        end
    end
        
        
    for UAV=1:swarmSize
        targ{UAV}(:) = target;
        y{UAV} = sim_GPS(x,UAV,navMemory{UAV},targ{UAV});
        [u{UAV},navMemory{UAV},targ{UAV}] = simNavDecision(y{UAV},u{UAV},navMemory{UAV},UAV,targ{UAV},pollution);
        x(:,UAV) = simMove(x(:,UAV),u{UAV},dt);
        pollution = cloudsamp(cloud,x(1,UAV),x(2,UAV),t);
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
y.DistanceToGoal = sqrt((y.Position(1)-targ(1))^2+(y.Position(2)-targ(2))^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [u,navMemory,targ] = simNavDecision(y,u,navMemory,UAV,targ,pollution)

navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];

switch navMemory.navState
    
    case 1, % state = 1 - drive to target
        
        u(1) = 10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
        u(2) = (3*pi/180) * (y.HeadingToGoal/(pi/2));
        
%         if y.DistanceToGoal < 40;
%             navMemory.navState = 3;
%         end
        
    case 2, % state = 2 - obstacle back away
        u(1) = 20 ;
        u(2) = 0.5*pi/180; 
       
    case 3, % state = 3 - random move for a bit
%         if UAV == 1
%             u = [10;0.005];
%         elseif UAV == 2
%             u = [10;0.007];
%         elseif UAV == 3
%             u = [10;0.01];
%         elseif UAV == 4
%             u = [10;0.05];
%         elseif UAV == 5
%             u = [10;0.1];
%         end
        u(1) = 20 ;
        u(2) = 0.5*pi/180;
        
        if pollution > 0.8 && pollution < 1.2
            navMemory.navState = 4;
            targ = y.Position;
        end
        
    case 4, % state = 4 - got there
        u(1) = 10;
        u(2) = 6*pi/180;
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