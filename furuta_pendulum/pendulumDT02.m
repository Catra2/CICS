function xk1 = pendulumDT02(xk, uk, Ts)
%% Discrete-time nonlinear dynamic model of a pendulum on a cart at time k
%#codegen
% 4 states (xk): 
%   cart position (z)
%   cart velocity (z_dot):
%   angle (theta): when 0
%   angular velocity (theta_dot): 

% Repeat application of Euler method sampled at Ts/M.
M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*pendulumCT02(xk1,uk);
end
% Note that we choose the Euler method (first oder Runge-Kutta method)
% because it is more efficient for plant with non-stiff ODEs.  You can
% choose other ODE solvers such as ode23, ode45 for better accuracy or
% ode15s and ode23s for stiff ODEs.  Those solvers are available from
% MATLAB.
