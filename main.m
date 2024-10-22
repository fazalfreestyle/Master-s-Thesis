% Main coordination script for running Robot 1 and Robot 2 in sequence

rosshutdown;  % Shutdown any previous ROS session

% Set up your laptop as the ROS master
master_IP = 'http://192.168.130.115:11311';  % Laptop's IP as ROS master
setenv('ROS_MASTER_URI', master_IP); 
setenv('ROS_IP', '192.168.130.115');  % Your laptop's IP
rosinit(master_IP);  % Initialize ROS with your laptop as the master

% Initialize separate nodes for Robot 1 and Robot 2
% node1 = ros.Node('/Robot1_node', '192.168.130.190');  % Robot 1 node
% node2 = ros.Node('/Robot2_node', '192.168.130.152');  % Robot 2 node

% Load the trained model for object classification (for Robot 2)
trainedModelPath = "C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Trained_Model\shape_classification_model_resnet50.mat";  % Update with your model path
load(trainedModelPath, 'net');  % Load the pre-trained model

% Sequential Task Execution Loop
for objectIndex = 1:6
    % Robot 1 handles Object objectIndex
    disp(['Starting Robot 1 Task for Object ', num2str(objectIndex)]);
    runRobot1Task(node1, objectIndex);  % Robot 1 task

    % Robot 2 performs its task after Robot 1 places the object
    disp(['Starting Robot 2 Task after Robot 1 placed Object ', num2str(objectIndex)]);
    runRobot2Task(node2, net);  % Robot 2 task
end

disp('Both robots have completed their tasks.');

% Shutdown ROS at the end
rosshutdown;
