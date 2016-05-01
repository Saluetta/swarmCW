function test
close all

mu = 1*pi/180;
v = 10;

X = [0;0;0.00];

dt = 2;

for i = 1:1000
    v = v;% + 0.05;
    mu = (2 + X(3,1)^2 ) / (0.5*(1 + X(3,1)^2)^1.5);
    
    if mu < 0
        mu = 0;
    end
    
    if mu > 6*pi/180
        mu = 6*pi/180;
    end
    
    if v > 20
        v = 20;
    end
    
    X = simMove(X,[v;mu],dt);
    store(:,i) = X(:,1);
    
    

end

plot(store(1,:), store(2,:), 'b')
plot(mod(store(3,:),2*pi), 'b')
hold on
plot(store(1,end), store(2,end), 'r*')

end


function [ X_next ] = simMove( X,U,dt )
%simMove given current state, control input and time step, this function
%returns the state at the next instant of time
%   Implements a simple 4th order Runge Kutta prediction

k1 = continuousDynamics(X,U);
k2 = continuousDynamics(X+k1*dt/2,U);
k3 = continuousDynamics(X+k2*dt/2,U);
k4 = continuousDynamics(X+k3*dt,U);

X_next = X + (k1 + 2*k2 + 2*k3 + k4)*dt/6;

end

% -------------------------------------------------------------------------
function [ X_dot ] = continuousDynamics( X,U )
%CONTINUOUSDYNAMICS simulates continuous dynamics of the system
%   Taken from the model of the UAV
%   X = [x;y;theta], U = [v;mu]
%   x' = v sin(theta)
%   y' = v cos(theta)
%   theta' = v mu

X_dot = zeros(3,1);
X_dot(1,1) = U(1,1) * sin( X(3,1) );
X_dot(2,1) = U(1,1) * cos( X(3,1) );
X_dot(3,1) = U(1,1) * U(2,1);

end