t = 0:2:1800;
mu = 6*pi/180*exp(-0.001*t);

mu(mu>6*pi/180) = 6*pi/180;
    
plot(t,mu)
