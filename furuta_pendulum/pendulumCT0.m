function dxdt = pendulumCT0(x, u)
%% Continuous-time nonlinear dynamic model of a rotary pendulum
% 4 states (x): 
%   rod angle (alpha)
%   rod velocity (alpha_dot): when positive, rod moves to right
%   angle (theta): when 0, pendulum is at upright position
%   angular velocity (theta_dot): when positive, pendulum counterclockwise
% 
% 2 inputs: (u)
%   u1: voltage when positive, rod CW 
%   u2: disturbance torque, positive when pend CCW
%#codegen
%% Obtain x, u and y
% x
m2 = 0.024;             % pendulum mass (kg)
m1 = 0.095;             % rod mass (kg)
L1 = 0.085;             % rod length (m)
L2 = 0.129;             % pendulum link length (m)
g  = 9.81;              % gravity of earth (m/s^2)
l1 = L1/2;              % rod radius to c.o.m. (m)
l2 = L2/2;              % pendulum radius to c.o.m. (m)
b1 = 0;                 % approximate viscous damping (N)
b2 = 0;
J2 = 0.5*m2*(l2)^2;     % pendulum inertia (kg.m^2)
J1 = 4e-6 + 0.6e-6;     % (motor + hub) rod inertia (kg.m^2)
Kt = 0.042;             % Torque constant (N.m/A)
R = 8.4;                % Motor resistance (Ohm)
v2t = Kt/R;             % Voltage to Torque converter

% Adjusted inertial terms for motion around pivot (not just c.o.m.)
j0 = J1 + m1*(l1^2) + m2*(L1^2); % equilibrium inertia
j2 = J2 + m2*(l2^2);
% Controller
alpha_dot = x(2);
theta = x(3);
theta_dot = x(4);
tau1 = u;           % input is torque, voltage conversion later
tau2 = 0;           % no external torques applied, pendantic inclusion
%% dxdt for pendulum try 2
G = [-j2*b1; m2*L1*l2*cos(theta)*b2; -(j2^2)*sin(2*theta);...
        -0.5*j2*m2*L1*l2*cos(theta)*sin(2*theta); j2*m2*L1*l2*sin(theta)];
H = [j2; -m2*L1*l2*cos(theta); 0.5*(m2^2)*(l2^2)*L1*sin(2*theta)];
den = j0*j2 + (j2^2)*sin(theta).^2 - (m2^2)*(L1^2)*(l2^2)*cos(theta).^2;

x1 = [  alpha_dot;
        theta_dot;
        alpha_dot*theta_dot;
        alpha_dot^2;
        theta_dot^2];

x2 = [  tau1;
        tau2;
        g];

L = [m2*L1*l2*cos(theta)*b1; -b2*(j0 + j2*sin(theta).^2); ...
        m2*L1*l2*j2*cos(theta)*sin(2*theta); -0.5*sin(2*theta)*(j0*j2+(j2^2)*sin(theta).^2); ...
        -0.5*(m2^2)*(L1^2)*(l2^2)*sin(2*theta)];
M = [-m2*L1*l2*cos(theta); j0 + j2*sin(theta).^2; ...
        -m2*l2*cos(theta)*(j0 + j2*sin(theta).^2)];

%% dxdt for pendulum
dxdt = [alpha_dot;
         (transpose(G)*x1 + transpose(H)*x2)/den;      %         ((-j2*b1*alpha_dot)+(m2*L1*l2*cos(theta)*b2*theta_dot)+(-j2^2*sin(2*theta)*theta_dot*alpha_dot)+(-0.5*j2*m2*L1*l2*cos(theta)*sin(2*theta)*alpha_dot^2)+(j2*m2*L1*l2*sin(theta)*theta_dot^2)+((j2*tau)+(-m2*L1*l2*cos(theta)*tau2)+(0.5*(m2^2)*(l2^2)*L1*sin(2*theta)*g)))/(j0*j2+(j2^2)*sin(theta).^2 - (m2^2)*(L1^2)*(l2^2)*cos(theta).^2);
         theta_dot;
         (transpose(L)*x1 + transpose(M)*x2)/den];     %         ((m2*L1*l2*cos(theta)*b1*alpha_dot)+(-b2*(j0+j2*sin(theta).^2)*theta_dot)+(m2*L1*l2*j2*cos(theta)*sin(2*theta)*theta_dot*alpha_dot)+(-0.5*sin(2*theta)*(j0*j2+(j2^2)*sin(theta).^2)*alpha_dot^2)+(-0.5*m2^2*L1^2*l2^2*sin(2*theta)*theta_dot^2)+(-m2*L1*l2*cos(theta)*tau)+((j0+j2*sin(theta).^2)*tau2)+(-m2*l2*sin(theta)*(j0+j2*sin(theta).^2)*g))/(j0*j2+(j2^2)*sin(theta).^2 - (m2^2)*(L1^2)*(l2^2)*cos(theta).^2)];
end