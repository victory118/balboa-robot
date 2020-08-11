% Inverted Pendulum Parameter File

% Physical parameters of the inverted pendulum known to the controller
P.m1 = 0.25;     % Mass of the pendulum, 0.25 kg
P.m2 = 1.0;      % Mass of the cart, 1.0 kg
P.ell = 0.5;    % Length of the rod, 0.5 m
P.g = 9.8;       % Gravity, 9.8 m/s**2
P.b = 0.05;      % Damping coefficient, 0.05 Ns

P.Rw = 0.08/2; % Radius of Pololu 80x10 mm wheel, m
P.mw = 0.02835; % Mass of Pololu 80x10 mm wheel, kg
P.Iw = (2*pi+2)/(2*pi+6)*2*P.mw*P.Rw^2; % Moment of Inertia of Pololu 80x10 mm wheel, kg*m**2
P.mb = 
P.Ib = 1/3*P.m1*P.ell^2;    % Moment of inertia of the pendulum (thin rod), kg*m**2

% parameters for animation
P.w = 0.5;       % Width of the cart, m
P.h = 0.15;      % Height of the cart, m
P.gap = 0.005;   % Gap between the cart and x-axis
P.radius = 0.06; % Radius of circular part of pendulum

% initial conditions
P.z0 = 0;         % initial position of cart in m
P.zdot0 = 0;      % initial velocity of cart in m/s
P.theta0 = 0;     % initial angle of rod in rad
P.thetadot0 = 0;  % initial angular velocity of rod in rad/sec

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% control saturation limits
P.F_max = 5; % Max Force, N
