% Load requirements for Furuta Pendulum configuration (QUBE2)

% pendulum equilibrium coordinate is (pi) and CCW rotation is positive

rng(1234)
M = 0;                  % Pendulum point-mass (kg)
mp = 0.024;             % pendulum mass (kg)
ma = 0.095;             % rod mass (kg)
la = 0.085;             % rod length (m)
lp = 0.129;             % pendulum link length (m)
g  = 9.81;              % gravity of earth (m/s^2)
b = 0.0015;             % approximate viscous damping (N)
J = 4.6e-6;             % (motor + hub) rod inertia (kg.m^2)
R = 8.4;                % motor resistance (Ohm)
Kt = 0.042;             % Motor torque constant (N.m/A ... i think?)
v2t = 2*Kt/R;           % volts-to-torque conversion factor (N.m/A)

alpha = J + (M + ma/3 + mp)*la^2;
gamma = (M + mp/2)*la*lp;
beta = (M + mp/3)*lp^2;
delta = (M + mp/2)*g*lp;
% Controller

N = 10; % Number of simulations
nx = 4; % Number of states
ny = 2; % Number of outputs
nu = 1; % Number of control inputs

% Different initial conditions for each simulation
rotLim = pi; % limit of rotation (+- rad) 
voltLim = 10;% voltage limitations of motor (+- volt)
x0 = zeros(nx,N); 
for i = 1:N
%     x0(1,i) = -rotLim/2 + 2*(rotLim/2)*rand;% arm Position
%     x0(2,i) = -(2) + 2*(2)*rand;% arm Velocity
     x0(3,i) = pi;%-(pi)*0.95 + 2*(pi)*0.95*rand;% Pendulum Angle
%     x0(4,i) = -(2) + 2*(2)*rand;% Pendulum Velocity
end
x0 = transpose(x0);

% Different set-points for each simulation
yref = zeros(N,ny);
for i = 1:N
    yref(i,1) = pi/6; % arm position
    yref(i,2) = 0; % Pendulum Angle
end
%% Controller creation and setup
controller = nlmpc(nx, ny, nu);
% controller.Optimization.SolverOptions.Algorithm = 'active-set';


% note: control horizon and sample time must be configured to give the 
%   optimization algorithm enough information to obtain a solution
%   or you will get fmincon errors for a week and hate your life
Ts = 0.05;
controller.Ts = Ts;
controller.PredictionHorizon = 10;
controller.ControlHorizon = 5;

% State function defined in custom file at /thesis/MATLAB_Examples
controller.Model.StateFcn = "pendulumDT0";
controller.Model.IsContinuousTime = false;
controller.Model.NumberOfParameters = 1;
controller.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];
controller.Weights.OutputVariables = [1 10];
controller.Weights.ManipulatedVariablesRate = 0.01;
controller.Weights.ManipulatedVariables = 2;
controller.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0; 0 0 1 0];
controller.OV(1).Min = -rotLim;
controller.OV(1).Max = rotLim;
controller.MV.Min = -voltLim; % volts or torque depending; check CT0
controller.MV.Max = voltLim;
controller.MV.ScaleFactor = 2*voltLim;
controller.OV(1).ScaleFactor = 2*rotLim;
controller.OV(2).ScaleFactor = 2*rotLim;

EKF = extendedKalmanFilter(@pendulumStateFcn,@pendulumMeasurementFcn);

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
%createParameterBus(controller,['mpc_furuta/Nonlinear MPC Controller'],'myBusObject',{Ts});
Duration = 10;
SL = Duration/Ts; % Sequence Length

%% Model test section
% Reset simulation
x = x0(:,i);
y = [x(1);x(3)];
EKF.State = x;
mv = 0;

mvHistory = zeros(SL,1);
xHistory = zeros(nx,SL+1); % Extra index to hold init. state
timerMPC = zeros(SL,1);

xHistory(:,1) = x; % Initial state

for ct = 1:SL
    tic
    disp("start")
            % Correct previous prediction
            xk = correct(EKF,y);
            % Compute optimal control moves
            [mv,nloptions] = nlmpcmove(controller,xk,mv,yref(i,:),...
                [],nloptions);

            % Predict prediction model states for the next iteration
            predict(EKF,[mv; Ts]);
            % Implement first optimal control move
            x = pendulumDT0(x,mv,Ts);
            mvHistory(ct) = mv;
            % Generate sensor data
            
            y = x([1 3]);% + randn(2,1)*0.01;
            % Save plant states
            xHistory(:,ct+1) = x;

end
x = (1:length(xHistory));
plot(x,rad2deg(xHistory([1 3],:)))
yline(rad2deg(yref(1,:)),'--')