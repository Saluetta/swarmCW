function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 3.6;
maxKK = 3600/dt;

% open new figure window
figure
axis([-1000 1000 -1000 1000])
hold on % so each plot doesn't wipe the predecessor


% simulation -> GROUND TRUTH
% uav -> SIMULATED UAV WITH NOISE

% state contains vectors [x; y; theta]
% UAVs are initialized around (0,0) and with random heading
simulation.state = zeros(3, maxKK + 1);
simulation.state(:, 1) = initializeSimulatedUAV();

% The simulated uavs are created
uav = UAV();

%uav.lastEstimatedVelocity = 0;

% 1 or -1
% uav.twistDirection = 2*round(rand()) - 1;

% uav.target = [500; 0];


% main simulation loop
% 1 HOUR
for kk = 1:maxKK,
    
    % time
    t = t + dt;
    
    % Simulation of measuring position with GPS
    uav = uav.simulateGPSreading(simulation.state(:, kk));
    
    % cheat - robot goes round in circles
    %x = [500*cos(0.01*t); 500*sin(0.01*t)];
    
    % take measurement
    p = cloudsamp(cloud, simulation.state(1, kk), simulation.state(2, kk),t);
    
%     [u, uav.lastEstimatedPosition, uav.lastEstimatedVelocity, uav.lastEstimatedHeading, ...
%         uav.stepsExploring] = simulate_decision(...
%         estimatedPosition, ...
%         uav.lastEstimatedPosition, ...
%         uav.lastEstimatedVelocity, ...
%         uav.lastEstimatedHeading, ...
%         uav.state, ...
%         uav.twistDirection, ...
%         uav.stepsExploring, ...
%         uav.target, ...
%         p, kk, dt, maxKK);

    [uav, u] = uav.simulateDecision(p, kk, dt);
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f', ...
        t, simulation.state(1, kk), simulation.state(2, kk), p))
        
    % plot robot location - ground truth
    plot(simulation.state(1, 1:kk), simulation.state(2, 1:kk))
    plot(simulation.state(1, kk), simulation.state(2, kk), 'ob')
    plot([simulation.state(1, kk), simulation.state(1, kk) + ...
        100*cos(pi/2 - simulation.state(3, kk))], [simulation.state(2, kk),...
        simulation.state(2, kk) + 100*sin(pi/2 - simulation.state(3, kk))], 'b');
   
    
    % plot robot location - estimation
    estimatedPositionUAV = uav.getEstimatedPosition;
    estimatedHeadingUAV = uav.getEstimatedHeading();
    
    plot(estimatedPositionUAV(1), estimatedPositionUAV(2), 'or')
    plot([estimatedPositionUAV(1), estimatedPositionUAV(1) + ...
        100*cos(pi/2 - estimatedHeadingUAV)], [estimatedPositionUAV(2), ...
        estimatedPositionUAV(2) + 100*sin(pi/2 - estimatedHeadingUAV)], 'r');
    
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    %pause(0.01)
    
    % execute decision
    simulation.state(:, kk + 1) = simMove(simulation.state(:,kk), u, dt);

end

function xnew = simMove(x,u,dt)
% runge kutta 4th order
k1 = f_continuous(x,u);
k2 = f_continuous(x+k1*dt/2,u);
k3 = f_continuous(x+k2*dt/2,u);
k4 = f_continuous(x+k3*dt,u);
xnew = x+(k1+2*k2+2*k3+k4)*dt/6;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot=f_continuous(x,u)
% simple point mass: x is [2D pos; 2D vel] and u is [2D acc]
% so xdot is just a rearrange
xdot = [    u(1)*sin(x(3)); 
            u(1)*cos(x(3)); 
            u(1)*u(2)];