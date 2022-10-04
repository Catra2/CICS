%% Generate training data for the LSTM
% Contents: This script configures an NLMPC to calculate control moves for
% a given model. The input-output data, along with timing characteristics,
% is saved to the current directory in a format compatible with the MATLAB
% LSTM training algorithm (state sequence --> appropriate move action). A
% paper was submitted to the CICS conference under the title of
% "Performance Analysis of LSTM-Based Nonlinear Model Predictive
% Controllers" that uses these scripts to generate data, train the LSTM,
% and make a comparison.

% HOW TO USE: 
%   1) edit the parameters to your preference in the config.json
%       file in this directory (defaulted to values used in the aforementioned
%       paper submission)
%   2) confirm the following files are in the same directory:
%       - pendulumStateFcn.m
%       - pendulumMeasurementFcn.m
%       - pendulumDT02.m
%       - pendulumCT02.m
%       
%       To train the LSTM, also ensure the following files are
%       in the path:
%       - trainingPartitions.m
%       - lstm_pendulum.m
%       - comparison_mpc_lstm.m
%
%       To get these files if you have stumbled across this script, email
%       the author if they're not on github: github.com/catra2/thesis
%   3) type "data_generator_qube" in the command window

% REQUIREMENTS: MATLAB Toolboxes: control, mpc, optimization, deep learning

% TODO: 1) create config.json and turn this script into
%          a callable function
%       2) partition most sections into subroutines, specifically the
%          data scrubbers

% Reproducability
seed = 1337;
rng(seed)

% Specify random initial conditions
N = 40; % Number of simulations
nx = 4; % Number of states
ny = 2; % Number of outputs
nu = 1; % Number of control inputs

if (rem(N,4)~=0)
    error("Your choice of N does not produce an integer for N/4")
    % The following data split section will produce an error if N/4 is not
    % an integer, as it is used to index the initial conditions
end

% Different random initial conditions for each simulation
rotLim = 0.7*pi; % +- limit of arm rotation (rad)
torqueLim = 0.8*0.042; % gain*Kt (max cont. torque = 22 mN for QUBE)
x0 = zeros(nx,N);
for i = 1:N
    x0(1,i) = -rotLim/10 + 2*(rotLim/10)*rand;% rotary Position (rad)
    x0(2,i) = -(1) + 2*(1)*rand;% rotary Velocity (rad/s)
    x0(3,i) = -(2*pi) + 4*(pi)*rand;% Pendulum Angle (rad)
    x0(4,i) = -(1) + 2*(1)*rand;% Pendulum Velocity (rad/s)
end

% Different set-points for each simulation
yref = zeros(N,ny);

% Split trials so that some pendulum set-points border unstable solutions
% this allows the LSTM training data to include information about
% how to respond when close to, but not exactly at, stable
% positions

for i = 1:(N/4)
    % unstable set-point (pend ~=k*pi) control actions
    yref(i,1) = -(0.5) + 2*(0.5)*rand; % rotary position (rad)
    yref(i,2) = -(0.1) + 2*(0.1)*rand; % Pendulum Angle (rad)
end

for i = ((N/4)+1):N
    % Most trials will provide data about stable configurations (pend=0)
    yref(i,1) = -(0.5) + 2*(0.5)*rand; % rotary position (rad)
    yref(i,2) = 0; % Pendulum Angle (rad)
end
%% Controller creation and setup
Ts = 0.04; % Sample time for NLMPC, EKF, and discrete Euler-method model
pHorizon = 25; % p
mHorizon = 25; % m is used in documentation

W_MVR = 0.1;
W_MV = 1;
W_OV = [2 6];

% In case an NLMPC was previously defined in the workspace
clear controller
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
controller.MV.Min = -torqueLim;
controller.MV.Max = torqueLim;

controller.MV.ScaleFactor = 2*torqueLim;
controller.OV(1).ScaleFactor = 2*rotLim;
controller.OV(2).ScaleFactor = 4*pi;

EKF = extendedKalmanFilter(@pendulumStateFcn,@pendulumMeasurementFcn);

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

% Save in case of required reset (see poor performance subsection below)
save controller

Duration = 5; % Simulated duration time from the perspective of the model
SL = Duration/Ts; % Sequence Length
move = cell(N,1);
response = cell(N,1);
runtimer = cell(N,1);
reference = cell(N,1);
mean_timer = cell(N,1);

% Iniitalize counters for new run
error_counter = 0;
failed_counter = 0;
deleted_counter = 0;

%% Data generator

for i = 1:N
    % Initialize simulation
    x = x0(:,i);
    y = [x(1);x(3)];
    EKF.State = x;
    mv = 0;

    mvHistory = zeros(SL,1);
    xHistory = zeros(nx,SL+1); % Extra index to hold init. state
    timerMPC = zeros(SL,1);

    xHistory(:,1) = x; % Initial state
    disp('0123456789012345678') % Random string to be deleted
    %% Simulation Loop
    try
        for ct = 1:SL
            % Print a progress report to the console, deleting the previous
            % one so you don't get a massive unreadable scrolling effect
            fprintf(1,['\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\bMv ' ...
                '[%03d/%d][%05.1fs]'],ct,SL,sum(timerMPC))
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

            % Update the state using user-defined model (x = xk+1)
            x = pendulumDT02(x,mv,Ts);

            % Generate sensor data
            y = x([1 3]) + randn(2,1)*0.0005; % encoder res. +- 0.175 (deg)

            % Store plant data
            xHistory(:,ct+1) = x;
            mvHistory(ct) = mv;
        end
        %% Storage
        % Get the total cost for the run
        mpcArmError(i) = sum(abs(xHistory(1,:) - yref(i,1))); %#ok<SAGROW>  arm cost
        mpcPendError(i) = sum(abs(xHistory(3,:) - yref(i,2))); %#ok<SAGROW>  pendulum cost

        fprintf('\nRun time for (%i/%i): %4.2f s, cost: ARM[%3.1f], PEND[%4.1f]\n',...
            i,N,SL*mean(timerMPC),mpcArmError(i),mpcPendError(i))

        move{i} = mvHistory';
        response{i} = xHistory(:,1:end-1); % Last one doesn't matter for train
        runtimer{i} = timerMPC(:);
        reference{i} = yref(i,:)';
        mean_timer{i} = sum(timerMPC);

    catch e
        % Try next configuration
        sprintf("Error on iteration %i, skipping",i)
        error_counter = error_counter + 1;
        errored_trials(error_counter) = i; %#ok<SAGROW>
        continue
    end

    %% Poor performance detection
    % Detect poor trial performance and reset the NLMPC controller
    if (mpcArmError(i) > 75)||(mpcPendError(i) > 450)
        % report and reset
        failed_counter = failed_counter + 1;
        failed_trials(failed_counter) = i; %#ok<SAGROW> shouldn't be too many
        fprintf("\nFlagged run [%d] for deletion (inaccurate)",i)
        fprintf(" ARM[%3.1f], PEND[%4.1f]\n",mpcArmError(i),mpcPendError(i))
        clear controller
        load controller.mat
    end
end
%% Data Scrubbing after N trials
% Empty the cell array for the indeces where failures were detected
for k = 1:failed_counter
    rtd = failed_trials(k); % run to delete
    runtimer{rtd} = [];
    response{rtd} = [];
    reference{rtd} = [];
    move{rtd} = [];
    mean_timer{rtd} = [];
end
for j = N:-1:1
    if isempty(response{j})
        % flag crap runs that made it through filter
        runtimer(j) = [];
        response(j) = [];
        reference(j) = [];
        move(j) = [];
        mean_timer(j) = [];
        deleted_counter = deleted_counter + 1;
        fprintf("Possibly deleted [%d] total trials",deleted_counter)
    end
end
disp(deleted_counter)
%% Analysis
% Inspect a few trials
for k = 1:3
    xx = linspace(1,Duration,length(move{k}));
    yy = response{k};
    if ~isempty(yy) % If fmincon error, skip
        figure
        yyaxis left
        plot(xx,yy([1 3],:))
        ylim([-(2*rotLim), (2*rotLim)])
        hold on
        yyaxis right
        plot(xx,move{k})
        ylim([-(torqueLim*2), (torqueLim*2)])
        legend('Arm','Pend','Voltage')
    end
end
%% Save
% TODO: format saving using date-time and other info, approximate
% pseudocode given below:

[year, month, day] = ymd(datetime);
[h, m, s] = hms(datetime);
success = N - nnz(cellfun(@isempty,move)); % Counter number of empty cells
directory = sprintf("%d_%d_%d%d_S%d_N%d",month,day,h,m,success,N);
mkdir(directory);

save(sprintf('%s/move.mat',directory),'move')
save(sprintf('%s/response.mat',directory),'response')
save(sprintf('%s/runtimer.mat',directory),'runtimer')
save(sprintf('%s/reference.mat',directory),'reference')
readmeCell = {"Parameter", "Value";
    "________","________";
    "Sample Time:", Ts;
    "pHorizon:", pHorizon;
    "mHorizon:", mHorizon;
    "Duration:", Duration;
    "Random Seed:", seed;
    "Arm Lim:", rotLim;
    "Input Lim:", torqueLim;
    "MV Weight:", W_MV;
    "MVR Weight:", W_MVR;
    "OV Weights:", W_OV};

writecell(readmeCell,sprintf('%s/MPC_info.txt',directory),'Delimiter','tab')
fprintf("Saved to %s",directory)
%%
disp('generator complete')
% run('lstm_pendulum.m')