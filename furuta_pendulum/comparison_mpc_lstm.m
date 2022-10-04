% Performance comparison for LSTM implementation (post-training)

nx = 4; % Number of states
ny = 2; % Number of outputs
nu = 1; % Number of control inputs

% Different initial conditions for each simulation
rotLim = 0.65*pi; % +- lim of rotor (rad) 
torqueLim = 0.8*0.042; % gain*Kt

Ts = 0.02;
pHorizon = 35; % p
mHorizon = 30; % m is used in documentation

W_MVR = 0.1;
W_MV = 1;
W_OV = [2 4];
x0 = [0;0;pi;0];
x=x0;
y = [x(1);x(3)];

controller = nlmpc(nx, ny, nu);
controller.Ts = Ts;
controller.PredictionHorizon = pHorizon;
controller.ControlHorizon = mHorizon;
controller.Model.StateFcn = "pendulumDT02";
controller.Model.IsContinuousTime = false;
controller.Model.NumberOfParameters = 1;
controller.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];
controller.Weights.OutputVariables = W_OV;
controller.Weights.ManipulatedVariablesRate = W_MVR;
controller.Weights.ManipulatedVariables = W_MV;
controller.OV(1).Min = -rotLim;
controller.OV(1).Max = rotLim;
controller.OV(2).Min = -6*pi;
controller.OV(2).Max = 6*pi;
controller.MV.Min = -torqueLim; % volts or torque depending; check CT0
controller.MV.Max = torqueLim;

controller.MV.ScaleFactor = 2*torqueLim;
controller.OV(1).ScaleFactor = 2*rotLim;
controller.OV(2).ScaleFactor = 4*pi;

EKF = extendedKalmanFilter(@pendulumStateFcn,@pendulumMeasurementFcn);

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
Duration = 5;
SL = Duration/Ts; % Sequence Length
EKF.State = x;
mv = 0;

yref = [0 0];
yseq = yref;

History = x;
mvHistory = zeros(Duration/Ts,1);
timerMPC = zeros((Duration/Ts),1);
timerLSTM = zeros((Duration/Ts),1);
    disp('0123456789012345678') % chars to delete on first run
% MPC

for ct = 1:(Duration/Ts)
        fprintf(1,'\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\bMv [%03d/%d][%05.1fs]',ct,SL,...
        sum(timerMPC))
    tic
    % Correct previous prediction
    xk = correct(EKF,y);
    % Compute optimal control moves
    [mv,nloptions] = nlmpcmove(controller,xk,mv,yref,[],nloptions);
    % Predict prediction model states for the next iteration
    predict(EKF,[mv; Ts]);
    % Implement first optimal control move
    timerMPC(ct) = toc;
    x = pendulumDT02(x,mv,Ts);
    mvHistory(ct) = mv;
    % Generate sensor data
    y = x([1 3]) + randn(2,1)*0.0001;
    % Save plant states
    History = [History x];
end
mean(timerMPC)

%% LSTM
% Reset simulation conditions to default

x = x0;
aiHistory = x;
aimvHistory = zeros(Duration/Ts,1);
%load('LSTM_QUBE1.mat') % bring in pre-trained network
for ct = 1:(Duration/Ts)
    tic

    sequence = vertcat(aiHistory,yseq');
    yseq = vertcat(yseq,yref); % Add another row for the next sequence

    % Compute optimal control moves
    action = predict(net,sequence, SequenceLength="shortest");
    aimv = action(end);
    timerLSTM(ct) = toc;

    % Implement first optimal control move
    x = pendulumDT02(x,aimv,Ts);
    aimvHistory(ct) = aimv;
    % Generate sensor data
    y = x([1 3]) + randn(2,1)*0.0001;
    % Save plant states
    aiHistory = [aiHistory x];
end
mean(timerLSTM)

%% Quantify Cost Performance
% MPC
mpcCartError = sum(abs(History(1,:) - yref(1)')); % note: allows yref ~= 0
mpcPendError = sum(abs(History(3,:) - yref(2)')); 

% LSTM
aiCartError = sum(abs(aiHistory(1,:) - yref(1)')); % note: allows yref ~= 0
aiPendError = sum(abs(aiHistory(3,:) - yref(2)'));

fprintf(1,"MPC Cart Error: %4.1f",mpcCartError);
fprintf(1,"LSTM Cart Error: %4.1f", aiCartError);

fprintf(1,"MPC Pendulum Error: %4.1f",mpcPendError);
fprintf(1,"LSTM Pendulum Error: %4.1f",aiPendError);
disp(a)
disp(b)
disp(c)
disp(d)
%% Visualization
figure
subplot(2,1,1)
hold on
plot(0:Ts:Duration,History(1,:))
plot(0:Ts:Duration,aiHistory(1,:))
yline(yref(1),'--b','Setpoint')
xlabel('Time (s)')
ylabel('\phi (rad)')
title('Arm')

subplot(2,1,2)
hold on
plot(0:Ts:Duration,History(3,:))
plot(0:Ts:Duration,aiHistory(3,:))
yline(yref(2),'--b','Setpoint')
xlabel('Time (s)')
ylabel('\theta (rad)')
title('Pendulum')
%% Functions
function nlmpcobj = load_controller()
    
end
