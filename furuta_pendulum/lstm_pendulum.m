% Configure an LSTM to add to a model controller
%% Load Data
% Generated from the data_generator.m script, appended 
% matrices stored as text files, (SLxNxChannels)
if ~exist('move')
    disp("Training data not found. Loading...")
    load('sim_qube1/move.mat');
    load('sim_qube1/response.mat');
    load('sim_qube1/reference.mat');
end
%% Data Preparation
% Application note: may need to add a scrubber for incomplete trials since
% the try/catch update to data_generator.m

N = numel(response); % number of simulations run
SL = size(response{1},2); % sequence length

% Input >>> response{1} (current state) << 4-chan input >>
% Output >>> move{1} (move based on current state) << 1-chan output >>
    % ~~~ Time Step ~~~~ (state update)
% Input >>> response{2} ...
% Output >>> move{2} ...

% Duplicate the reference signal (2x1) to all columns (2xSL)
for j = 1:N
    for i = 1:SL
        reference{j}(1,i) = reference{j}(1,1);
    end
end

% Assign features to a single data structure for the LSTM to read
data = cell(1,N);
for i = 1:N
    data{i} = vertcat(response{i}, reference{i});
end
labels = move;

numChannels = size(data{1},1);
numObservations = N;
numResponses = size(labels{1},1);

%% Prepare Data for LSTM
[idxTrain,idxValidation,idxTest] = trainingPartitions(numObservations, ...
    [0.8 0.1 0.1]);


XTrain = data(idxTrain);
XValidation = data(idxValidation);
XTest = data(idxTest);

TTrain = labels(idxTrain);
TValidation = labels(idxValidation);
TTest = labels(idxTest);

%% Define LSTM Architecture

layers = [ ...
    sequenceInputLayer(numChannels, Normalization="zscore")
    lstmLayer(200, OutputMode="sequence")
    lstmLayer(200, OutputMode="sequence")
    lstmLayer(100, OutputMode="sequence")
    lstmLayer(100, OutputMode="sequence")
    lstmLayer(100, OutputMode="sequence")
    fullyConnectedLayer(numResponses)
    regressionLayer];

%% Configure Training Options
options = trainingOptions("adam", ...
    MaxEpochs=100, ...
    ValidationData={XValidation TValidation}, ...
    OutputNetwork="best-validation-loss", ...
    LearnRateSchedule='piecewise', ...
    LearnRateDropPeriod=50, ...
    LearnRateDropFactor=0.5, ...
    InitialLearnRate=0.001, ...
    SequenceLength="shortest", ...
    Plots="training-progress", ...
    Verbose= false);

%% Train
net = trainNetwork(XTrain, TTrain, layers, options);

%% Test
tic
YTest = predict(net,XTest, SequenceLength="shortest");
toc
save('LSTM_QUBE1.mat','net')
readmeCellLSTM = {"Parameter", "Value";
                "________","________";
                "Layers:", 7;
                "Epochs:", 200;
                "LR:", 0.004;
                "Algorithm:", 'Adam';
                "Normalization:", 'z-score'};

writecell(readmeCellLSTM,'sim_qube1/LSTM_info.txt','Delimiter','tab')