%% Script to automate standardization (or normalization)

% Read in the reference training data (not transformed)
refTrainingData = readtable("D:\Users\Saian\Workspace\NeuralNetworks\FFNN\Iteration90\TrainingData.csv", "VariableNamingRule", "preserve");

% Get the preProcessArray
preProcessArray = fnMakePreProcessArray(table2array(refTrainingData), 'S');

%% Function for producing an array for data transformation to be used in Simulink
function preProcessArray = fnMakePreProcessArray(refTrainingData, processMode)

    % Get the number of inputs
    nInputs = size(refTrainingData, 2) - 1;
    preProcessArray = zeros(2, nInputs);

    switch processMode

        case 'S'

            % Get the mean and std of each column and store (1. Mean, 2.
            % Std)
            for i = 1:nInputs

                preProcessArray(1,i) = mean(refTrainingData(:,i));
                preProcessArray(2,i) = std(refTrainingData(:,i));

            end

        case 'N'

            % Get the min and max of each column and store (1. Min, 2.
            % Max)
            for i = 1:nInputs

                preProcessArray(1,i) = min(refTrainingData(:,i));
                preProcessArray(2,i) = max(refTrainingData(:,i));

            end


        otherwise

            return;

    end


end