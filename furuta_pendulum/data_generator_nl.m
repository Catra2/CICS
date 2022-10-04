%% Generate training data for the LSTM
% Reproducability
rng(1234)

% Specify random initial conditions 
N = 1000; % Number of simulations
nx = 4; % Number of states
ny = 2; % Number of outputs
nu = 1; % Number of control inputs

% Different initial conditions for each simulation
x0 = zeros(nx,N); 
for i = 1:N
    x0(1,i) = -2 + 2*(2)*rand;% Cart Position
    % x0(2,i) = -(2) + 2*(2)*rand;% Cart Velocity
    x0(3,i) = -(pi)*0.95 + 2*(pi)*0.95*rand;% Pendulum Angle
    % x0(4,i) = -(2) + 2*(2)*rand;% Pendulum Velocity
end

% Different cart set-points for each simulation
yref = zeros(N,ny);
for i = 1:N
    yref(i,1) = -2 + 4*rand; % Cart position
    yref(i,2) = -0.05 + 2*(0.05)*rand; % Pendulum Angle (+-3 degrees)
end
%% Controller creation and setup
controller = nlmpc(nx, ny, nu);

Ts = 0.1;
controller.Ts = Ts;
controller.PredictionHorizon = 10;
controller.ControlHorizon = 5;

% State function defined in custom file at /thesis/MATLAB_Examples
controller.Model.StateFcn = "pendulumDT0";
controller.Model.IsContinuousTime = false;
controller.Model.NumberOfParameters = 1;
controller.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];
controller.Weights.OutputVariables = [2 8];
controller.Weights.ManipulatedVariablesRate = 0.1;
controller.OV(1).Min = -25;
controller.OV(1).Max = 25;
controller.MV.Min = -100;
controller.MV.Max = 100;

EKF = extendedKalmanFilter(@pendulumStateFcn,@pendulumMeasurementFcn);
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
Duration = 15;
SL = Duration/Ts; % Sequence Length
move = cell(N,1);
response = cell(N,1);
runtimer = cell(N,1);
reference = cell(N,1);
mean_timer = cell(N,1);

%% Start data generator loop
% Initialize a cancel button
f = waitbar(0,'1','Name','Data Generator',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

setappdata(f,'canceling',0);
generatorStart = tic;
for i = 1:N
    
    % Cancel button pressed
    if getappdata(f,'canceling')
        break
    end

    % Time estimator for generator
    % mean_time = mean(cell2mat(mean_timer));
    % estimated_total_time = mean_time*N;
    secs = toc(generatorStart);
    %[h,m,s] = hms(secs);
    waitbar(i/N,f,sprintf('Run (%i/%i), Elapsed: %i min',i,N, secs/60.0))

    % Reset simulation
    x = x0(:,i); % [0;0;-pi;0];
    y = [x(1);x(3)];
    EKF.State = x;
    mv = 0;
    
    mvHistory = zeros(SL,1);
    xHistory = zeros(nx,SL+1); % Extra index to hold init. state
    timerMPC = zeros(SL,1);
    
    xHistory(:,1) = x; % Initial state
    try
        for ct = 1:SL
            
            simStart = tic;
            % Correct previous prediction
            xk = correct(EKF,y);
            % Compute optimal control moves
            [mv,nloptions] = nlmpcmove(controller,xk,mv,yref(i,:),...
                [],nloptions);
            % Predict prediction model states for the next iteration
            predict(EKF,[mv; Ts]);
            % Implement first optimal control move
            timerMPC(ct) = toc(simStart);
            x = pendulumDT0(x,mv,Ts);
            mvHistory(ct) = mv;
            % Generate sensor data
            y = x([1 3]) + randn(2,1)*0.01;
            % Save plant states
            xHistory(:,ct+1) = x;
        end
    catch
        % Try next configuration
        sprintf("Invalid x0 on iteration %i, skipping",i)
        continue
    end
    fprintf('Run time for (%i/%i): %4.2f s\n',i,N,SL*mean(timerMPC))
    move{i} = mvHistory';
    response{i} = xHistory(:,1:end-1); % Last one doesn't matter for train
    runtimer{i} = timerMPC(:);
    reference{i} = yref(i,:)';
    mean_timer{i} = sum(timerMPC);
end
%% Save (comment for overwrite safety; don't ask me how I know)
% save('sim_yref/move.mat','move')
% save('sim_yref/response.mat','response')
% save('sim_yref/timer.mat','runtimer')
% save('sim_yref/reference.mat','reference')
% delete(f)
% disp('generator complete')