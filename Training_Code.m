% Define dataset paths
cuboidFolder = 'C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Cuboid';
cylinderFolder = 'C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Cylinder';
outputModelPath = 'C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Trained_Model\shape_classification_model.mat';

% Load images from folders
imdsCuboid = imageDatastore(cuboidFolder, 'LabelSource', 'foldernames', 'IncludeSubfolders', true);
imdsCylinder = imageDatastore(cylinderFolder, 'LabelSource', 'foldernames', 'IncludeSubfolders', true);

% Combine datastores
imds = imageDatastore(cat(1, imdsCuboid.Files, imdsCylinder.Files), 'Labels', categorical([repmat({'Cuboid'}, numel(imdsCuboid.Files), 1); repmat({'Cylinder'}, numel(imdsCylinder.Files), 1)]));

% Split data into training and validation sets
[trainImds, valImds] = splitEachLabel(imds, 0.8, 'randomized');

% Resize the images to 224x224 to match the network input layer
inputSize = [224 224 3];  % Input size for the network
augTrainImds = augmentedImageDatastore(inputSize(1:2), trainImds);
augValImds = augmentedImageDatastore(inputSize(1:2), valImds);

% Define a complex neural network
layers = [
    imageInputLayer(inputSize, 'Name', 'input')
    convolution2dLayer(3, 64, 'Padding', 'same', 'Name', 'conv_1')
    batchNormalizationLayer('Name', 'batchnorm_1')
    reluLayer('Name', 'relu_1')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'maxpool_1')

    convolution2dLayer(3, 128, 'Padding', 'same', 'Name', 'conv_2')
    batchNormalizationLayer('Name', 'batchnorm_2')
    reluLayer('Name', 'relu_2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'maxpool_2')

    convolution2dLayer(3, 256, 'Padding', 'same', 'Name', 'conv_3')
    batchNormalizationLayer('Name', 'batchnorm_3')
    reluLayer('Name', 'relu_3')
    fullyConnectedLayer(2, 'Name', 'fc') % 2 classes: Cuboid, Cylinder
    softmaxLayer('Name', 'softmax')
    classificationLayer('Name', 'classOutput')
    ];

% Add Early Stopping based on validation accuracy with a patience of 5
validationPatience = 5;
options = trainingOptions('adam', ...
    'MaxEpochs', 30, ...
    'InitialLearnRate', 1e-4, ...
    'ValidationData', augValImds, ...
    'ValidationFrequency', 10, ...
    'Verbose', true, ...
    'ExecutionEnvironment', 'gpu', ...  % Use GPU for faster training
    'Plots', 'training-progress', ...
    'MiniBatchSize', 16, ...  % Adjust the batch size for better generalization
    'Shuffle', 'every-epoch', ...
    'ValidationPatience', validationPatience, ...
    'OutputFcn', @(info)stopIfNoImprovement(info, validationPatience));

% Train the neural network
net = trainNetwork(augTrainImds, layers, options);

% Save the trained model
save(outputModelPath, 'net');
disp('Model training completed and saved successfully!');

% Callback function for early stopping
function stop = stopIfNoImprovement(info, validationPatience)
    stop = false;
    % Ensure that ValidationLoss exists and is non-empty
    if info.Epoch > 1 && isfield(info, 'ValidationLoss') && ~isempty(info.ValidationLoss)
        % Check if validation loss has not improved for "validationPatience" epochs
        if info.ValidationLoss(end) >= min(info.ValidationLoss) && info.Epoch >= validationPatience
            stop = true;
        end
    end
end
