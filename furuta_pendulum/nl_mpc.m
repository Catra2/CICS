% Nonlinear MPC: Inverted Cart-Pendumlum

controller = nlmpc(4, 2, 1);

Ts = 0.1;
controller.Ts = Ts;
controller.PredictionHorizon = 10;
controller.ControlHorizon = 5;

% State function defined in custom file 
controller.Model.StateFcn = "pendulumDT0";
controller.Model.IsContinuousTime = false;
controller.Model.NumberOfParameters = 1;
controller.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];

controller.Weights.OutputVariables = [3 3];
controller.Weights.ManipulatedVariablesRate = 0.1;
controller.OV(1).Min = -10;
controller.OV(1).Max = 10;
controller.MV.Min = -100;
controller.MV.Max = 100;

x0 = [0.1;0.2;-pi/2;0.3];
u0 = 0.4;
validateFcns(controller, x0, u0, [], {Ts});

EKF = extendedKalmanFilter(@pendulumStateFcn,@pendulumMeasurementFcn);

x = [0;0;-pi;0];
y = [x(1);x(3)];
EKF.State = x;
mv = 0;

yref = [0 0];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};


Duration = 10;
xHistory = x;
mvHistory = zeros(Duration/Ts,1);
timerMPC = zeros((Duration/Ts),1);
for ct = 1:(Duration/Ts)
    tic
    % Correct previous prediction
    xk = correct(EKF,y);
    % Compute optimal control moves
    [mv,nloptions] = nlmpcmove(controller,xk,mv,yref,[],nloptions);
    % Predict prediction model states for the next iteration
    predict(EKF,[mv; Ts]);
    % Implement first optimal control move
    timerMPC(ct) = toc;
    x = pendulumDT0(x,mv,Ts);
    mvHistory(ct) = mv;
    % Generate sensor data
    y = x([1 3]) + randn(2,1)*0.01;
    % Save plant states
    xHistory = [xHistory x];
end

figure
subplot(2,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time')
ylabel('z')
title('cart position')
subplot(2,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('zdot')
title('cart velocity')
subplot(2,2,3)
plot(0:Ts:Duration,xHistory(3,:))
xlabel('time')
ylabel('theta')
title('pendulum angle')
subplot(2,2,4)
plot(0:Ts:Duration,xHistory(4,:))
xlabel('time')
ylabel('thetadot')
title('pendulum velocity')