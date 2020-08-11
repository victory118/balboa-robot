% inverted pendulum - parameter file for hw8
% addpath ./.. % adds the parent directory to the path
balboaParam % general pendulum parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       PD Control: Time Design Strategy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
tr_th = 0.35;          % Rise time for inner loop (theta)
zeta_th = 0.707;       % Damping Coefficient for inner loop (theta)
M = 25.0;              % Time scale separation between inner and outer loop
zeta_z = 0.707;        % Damping Coefficient fop outer loop (z)

% saturation limits
P.error_max = 1;        		  % Max step size,m
P.theta_max = 30.0*pi/180.0;  % Max theta, rads

%---------------------------------------------------
%                    Inner Loop
%---------------------------------------------------
% parameters of the open loop transfer function
b0_th = -1.0/(P.m1*(P.ell/6.0) + P.m2*(2.0*P.ell/3.0));
a1_th = 0.0;
a0_th = -(P.m1+P.m2)*P.g/(P.m1*(P.ell/6.0) + P.m2*(2.0*P.ell/3.0));

% coefficients for desired inner loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
wn_th = 2.2/tr_th;     % Natural frequency
alpha1_th = 2.0*zeta_th*wn_th;
alpha0_th = wn_th^2;

% compute gains
% Delta(s) = s^2 + (a1 + b0*kd)*s + (a0 + b0*kp)
P.kp_th = (alpha0_th-a0_th)/b0_th;
P.kd_th = (alpha1_th-a1_th)/b0_th;
P.DC_gain = P.kp_th/((P.m1+P.m2)*P.g+P.kp_th);

%---------------------------------------------------
%                    Outer Loop
%---------------------------------------------------
% coefficients for desired outer loop
% Delta_des(s) = s^2 + alpha1*s + alpha0 = s^2 + 2*zeta*wn*s + wn^2
tr_z = M*tr_th;  % desired rise time, s
wn_z = 2.2/tr_z;  % desired natural frequency

% compute gains
a  = -(wn_z^2)*sqrt(2.0*P.ell/(3.0*P.g));
b = (a - 2.0*zeta_z*wn_z)*sqrt(2.0*P.ell/(3.0*P.g));
P.kd_z = b/(1-b);
P.kp_z = a*(1+P.kd_z);

fprintf('\t DC_gain: %f\n', P.DC_gain)
fprintf('\t kp_th: %f\n', P.kp_th)
fprintf('\t kd_th: %f\n', P.kd_th)
fprintf('\t kp_z: %f\n', P.kp_z)
fprintf('\t kd_z: %f\n', P.kd_z)



