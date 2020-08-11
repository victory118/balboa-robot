clc
close all
clear all

balboaParamHW8;  % load parameters

% instantiate pendulum, controller, and reference input classes
alpha = 0;
balboa = balboaDynamics(alpha,P);  
controller = balboaController(P);  
reference = signalGenerator(0.5, 0.02);  
disturbance = signalGenerator(0.0, 0);  

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = balboaAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = balboa.h();

sim_idx = 0;
plot_step = P.t_plot/P.Ts;

while t <= P.t_end
    % update control and dynamics at faster simulation rate
    r = reference.square(t);
    d = disturbance.step(t);
    n = [0; 0];  % noise
    x = balboa.state;
    u = controller.update(r, x);  % Calculate the control value
    
    % update animation and data plots at slower plot/animation rate
    if mod(sim_idx,plot_step) == 0
        animation.update(balboa.state);
        dataPlot.update(t, r, balboa.state, u);
    end
    
    y = balboa.update(u + d);  % Propagate the dynamics
    t = t + P.Ts; % advance time by Ts
    sim_idx = sim_idx + 1;
end

% while t < P.t_end  
%     % Propagate dynamics in between plot samples
%     t_next_plot = t + P.t_plot;
%     while t < t_next_plot % updates control and dynamics at faster simulation rate
%         r = reference.square(t);
%         d = disturbance.step(t);
%         n = [0; 0];  % noise
%         x = pendulum.state;
%         u = controller.update(r, x);  % Calculate the control value
%         y = pendulum.update(u + d);  % Propagate the dynamics
%         t = t + P.Ts; % advance time by Ts
%     end
%     % update animation and data plots
%     animation.update(pendulum.state);
%     dataPlot.update(t, r, pendulum.state, u);
% end


