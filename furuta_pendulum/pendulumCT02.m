function dxdt = pendulumCT02(x, u)
%#codegen
M  = 0;                  % Pendulum point-mass (kg)
mp = 0.024;             % pendulum mass (kg)
ma = 0.095;             % rod mass (kg)
la = 0.085;             % rod length (m)
lp = 0.129;             % pendulum link length (m)
g  = 9.81;              % gravity of earth (m/s^2)
b  = 0.001;             % approximate viscous damping (N)
J  = 4.6e-6;            % (motor + hub) rod inertia (kg.m^2)
R = 8.4;                  % motor resistance (Ohm)
Kt = 0.042;             % Motor torque constant (N.m/A ... i think?)
v2t = 2*Kt/R;             % volts-to-torque conversion factor (N.m/A)

alpha = J + (M + ma/3 + mp)*la^2;
gamma = (M + mp/2)*la*lp;
beta = (M + mp/3)*lp^2;
delta = (M + mp/2)*g*lp;

phi_dot = x(2);
theta = x(3);
theta_dot = x(4);

tau_phi = u - b*phi_dot;    % total torque = motor voltage - friction
tau_theta = 0;                  % no external torques applied
%% Equations of Motion
num_phi = beta*gamma*(sin(theta).^2-1)*sin(theta)*phi_dot^2-2*beta^2*...
    cos(theta)*sin(theta)*phi_dot*theta_dot + beta*gamma*sin(theta)*theta_dot^2 - ...
    gamma*delta*cos(theta)*sin(theta) + beta*tau_phi -...
    gamma*cos(theta)*tau_theta;

num_theta = beta*(alpha + beta*sin(theta).^2)*cos(theta)*sin(theta)*...
    phi_dot^2 + 2*beta*gamma*(1 - sin(theta).^2)*sin(theta)*...
    phi_dot*theta_dot - (gamma^2)*cos(theta)*sin(theta)*theta_dot^2 + ...
    delta*(alpha+beta*sin(theta).^2)*sin(theta) - gamma*cos(theta)*tau_phi...
    + (alpha + beta*sin(theta).^2)*tau_theta;

den = alpha*beta - (gamma^2) + (beta^2 + gamma^2)*sin(theta).^2;
%% dxdt for pendulum
dxdt = [phi_dot;
         num_phi/den;
         theta_dot;
         num_theta/den];     
end