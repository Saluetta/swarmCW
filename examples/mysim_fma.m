function mysim_fma

% initial state
x = [0;0];

% time step
dt = 0.05;

% time loop
for kk=1:1000,
    
    % thruster pulse
    if kk<100,
        u = 5*9.81;
    else
        u = -9.81;
    end
    
    % propagate state
    x = f_rk4(x,u,dt);
    
    % store
    xs(:,kk) = x;
    
end

% plot position
plot(dt*(1:1000),xs(1,:))
xlabel('Time (s)')
ylabel('Position')

function xnew=f_rk4(x,u,dt)
% runge kutta
k1 = f_continuous(x,u);
k2 = f_continuous(x+k1*dt/2,u);
k3 = f_continuous(x+k2*dt/2,u);
k4 = f_continuous(x+k3*dt,u);
xnew = x+(k1+2*k2+2*k3+k4)*dt/6;

function xdot=f_continuous(x,u)
% dynamics of SHM
m = 1; % mass
xdot = [x(2); u/m];
