function sim_swarm_ravens
load 'cloud1.mat'

% time and time step
t = 0;
dt = 3.6;

Num_agents = 5;
x = zeros(3,Num_agents);
v = 10;
mu = 0;

for agent = 1:Num_agents,
    navMemory{agent}.lastPos = [0;0];
    navMemory{agent}.navState = 1;
    u{agent} = [v;mu];
    navMemory{agent}.velCommands = [u{agent}(1),u{agent}(2)];
    targ{agent} = [500;0];
end

figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    t = t + dt;
    
    for agent=1:Num_agents
        y{agent} = sim_GPS(x,agent,navMemory{agent},targ{agent});
        [u{agent},navMemory{agent}] = simNavDecision(y{agent},u{agent},navMemory{agent});
        p = cloudsamp(cloud,x(1,agent),x(2,agent),t);
        if p > 0.8 && p < 1.2
            targ{agent} = y{agent}.Position;
        end       
    end
    
    cla
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),p))
    plot(x(1,:),x(2,:),'bo')
    for agent=1:Num_agents
        plot(targ{agent}(1),targ{agent}(2),'gs')
    end
    cloudplot(cloud,t)
    cloudsamp(cloud,x(1),x(2),t);
    pause(0.1)
    
    for agent=1:Num_agents,
        % execute decision
        x(:,agent) = simMove(x(:,agent),u{agent},navMemory{agent},dt);
    end
    
end

function [u, navMemory] = simNavDecision(y,u,navMemory)
navMemory.lastPos = y.Position;
navMemory.velCommands = [u(1),u(2)];
u(1) = 10 * ((pi/2 - abs(y.HeadingToGoal))/(pi/2));
u(2) =  (3*pi/180) * (y.HeadingToGoal/(pi/2));

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
xdot = [u(1)*sin(x(3));...
        u(1)*cos(x(3));...
        u(1)*u(2)];

function y = sim_GPS(x,aa,navMemory,targ)
y.Position = x(1:2,aa) + 3*randn(2,1); % New stimated position plus noise
y.Heading = atan2(y.Position(1)-navMemory.lastPos(1),y.Position(2)-navMemory.lastPos(2));% New stimated orientation
y.HeadingToGoal = atan2(targ(1)-navMemory.lastPos(1),targ(2)-navMemory.lastPos(2)) - y.Heading;