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

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt +100;
    
    % cheat - robot goes round in circles
    x = [500*cos(0.01*t); 500*sin(0.01*t)];
    
    % take measurement
    p = cloudsamp(cloud,x(1),x(2),t);
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, x(1),x(2),p))
        
    % plot robot location
    plot(x(1),x(2),'o')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end


% For dynamics> 1.90509 kg