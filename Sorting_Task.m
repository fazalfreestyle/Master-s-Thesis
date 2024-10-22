% Load the URDF file and create a RigidBodyTree object
ned = importrobot("C:\Users\fazal\Downloads\ned2\ned2\niryo_ned2_gripper1_n_camera.urdf");

% Define end effector offset (if any)
eeoffset = 0;
eeBody = robotics.RigidBody("end_effector");
setFixedTransform(eeBody.Joint, trvec2tform([eeoffset, 0, 0]));
addBody(ned, eeBody, "tool_link");

% Create an inverse kinematics object for the robot
ik = inverseKinematics("RigidBodyTree", ned);
weight = [0.1, 0.1, 0, 1, 1, 1];
initialguess = ned.homeConfiguration;

% Load the pre-trained network
trainedModelPath = 'C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Trained_Model\shape_classification_model_resnet50.mat';
load(trainedModelPath, 'net');  % Load the pre-trained model

% ROS initialization with connection check
rosshutdown;
robot1_IP = 'http://192.168.130.152:11311';
user_IP = '192.168.130.115';
setenv('ROS_MASTER_URI', robot1_IP);  % IP of the Ned
setenv('ROS_IP', user_IP);  % IP of the computer

try
    rosinit(robot1_IP);  % Initialize ROS connection
    disp('ROS initialized successfully.');
catch
    error('Could not initialize ROS. Please check the IP addresses and ROS setup.');
end

% Subscribe to the current joint states and camera feed
jointStateSub = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");
cameraSub = rossubscriber("/niryo_robot_vision/compressed_video_stream", "sensor_msgs/CompressedImage");

% Gripper control setup
NedCmd2 = rospublisher("/niryo_robot_tools_commander/action_server/goal");
CmdMsg2 = rosmessage(NedCmd2);

% Positions and orientations
home_position = [0.150, 0, 0.200];          % Home position
home_orientation = [0, 0.77, 0];            % Home orientation
work_pov_position = [0, -0.310, 0.150];     % Camera POV position
work_object_position = [0, -0.415, 0.100];  % Work environment object position
work_above_object_position = [0, -0.415, 0.150];  % Work environment above object position
cuboid_box_position = [-0.100, 0.300, 0.160]; % Cuboid box position
cylinder_box_position = [0.100, 0.300, 0.160]; % Cylinder box position
orientation = [0, 1.540, 0];               % Same orientation for all

% Move to Home Position
disp('Moving to Home Position...');
move(ned, home_position, home_orientation, jointStateSub);

% Move to Work Environment POV Camera Position
disp('Moving to Work Environment POV Camera Position...');
move(ned, work_pov_position, orientation, jointStateSub);

% Ensure the camera feed is active before capturing
cameraActiveSub = rossubscriber("/niryo_robot_vision/video_stream_is_active");
cameraActiveMsg = receive(cameraActiveSub, 10);
if cameraActiveMsg.Data
    % Capture the image from the camera
    disp('Capturing image...');
    try
        cameraMsg = receive(cameraSub, 10);  % Timeout of 10 seconds
        
        % Extract and decode the compressed image data (in JPEG format)
        imgData = cameraMsg.Data;  % Image data is of type uint8
        tempFile = tempname;  % Create a temporary file
        fid = fopen([tempFile, '.jpg'], 'w');  % Open the temp file to write the image
        fwrite(fid, imgData);  % Write the image data to file
        fclose(fid);  % Close the file
        img = imread([tempFile, '.jpg']);  % Read the image back using imread
        imgResized = imresize(img, [224, 224]);  % Resize the image to the network input size
        
        % Display the captured image
        imshow(img);
        
        % Classify the image using the trained neural network
        disp('Classifying the captured image...');
        predictedLabel = classify(net, imgResized);
        disp(['Predicted Shape: ', char(predictedLabel)]);
    catch
        error('Error capturing image: Ensure the camera is active and connected.');
    end
else
    error('Camera feed is not active.');
end

% Move to the above object position before going to the object
disp('Moving to above object position...');
move(ned, work_above_object_position, orientation, jointStateSub);

% Move to the object position to grab the object
disp('Moving to object position...');
move(ned, work_object_position, orientation, jointStateSub);

% Close the gripper to grab the object
controlGripper(NedCmd2, CmdMsg2, 2);  % Close the gripper

% Move back to above object position
disp('Moving back above object...');
move(ned, work_above_object_position, orientation, jointStateSub);

% Based on the prediction, decide where to place the object
if strcmp(char(predictedLabel), 'Cylinder')
    disp('Moving to Cylinder Box...');
    move(ned, cylinder_box_position, orientation, jointStateSub);
else
    disp('Moving to Cuboid Box...');
    move(ned, cuboid_box_position, orientation, jointStateSub);
end

% Open the gripper to release the object
controlGripper(NedCmd2, CmdMsg2, 1);  % Open the gripper

% Move back to home position
disp('Moving back to Home Position...');
move(ned, home_position, home_orientation, jointStateSub);

% Shut down ROS
rosshutdown;

%% Function to move the robot to the given position and orientation
function move(ned, vect, eul, jointStateSub)
    ik = inverseKinematics("RigidBodyTree", ned);
    weight = [0.1 0.1 0 1 1 1];
    initialguess = ned.homeConfiguration;

    % Convert position and orientation to transformation matrix
    postform = trvec2tform(vect);
    eultform = eul2tform(eul, 'ZYX');
    tform = postform * eultform;

    % Solve inverse kinematics
    configSoln = ik("tool_link", tform, weight, initialguess);
    cell = struct2cell(configSoln);
    Joint = cell(2,:,:);
    matrixJoints = cell2mat(Joint);

    % Create ROS publishers and send the trajectory to the robot
    NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
    CmdMsg = rosmessage(NedCmd);
    CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');

    % Fill in the joint positions
    for i = 1:6
        CmdPoint.Positions(i) = matrixJoints(i);
    end

    % Set velocities, accelerations, and time to complete the movement
    CmdPoint.Velocities = zeros(1, 6);
    CmdPoint.Accelerations = zeros(1, 6);
    CmdPoint.TimeFromStart = ros.msg.Duration(6);  % Increase duration to 6 seconds

    % Set the message header and joint names
    CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
    CmdMsg.Points = CmdPoint;

    % Send the command to the robot
    send(NedCmd, CmdMsg);

    % Wait until the robot reaches the target position by comparing joint states
    maxWaitTime = 20;  % Increase timeout to 20 seconds
    tolerance = 0.05;  % Increase tolerance to 5 cm
    startTime = tic;

    while true
        % Get the current joint states from the subscriber
        currentState = receive(jointStateSub, 10);  % Timeout after 10 seconds
        currentJoints = currentState.Actual.Positions;

        % Compare current joint positions to the target joint positions
        if all(abs(currentJoints - matrixJoints) < tolerance)
            disp('Target position reached.');
            break;
        end

        % Check if the loop has run for too long
        if toc(startTime) > maxWaitTime
            disp('Timeout: Target not reached within allowed time.');
            break;
        end

        pause(0.1);  % Small pause to avoid overloading the system
    end
end

%% Gripper control function
function controlGripper(NedCmd2, CmdMsg2, action)
    % Create a gripper command message
    CmdGoal = rosmessage('niryo_robot_tools_commander/ToolGoal');
    
    % Set Tool ID and Command Type
    CmdGoal.Cmd.ToolId = 11;  % Gripper tool ID
    CmdGoal.Cmd.CmdType = action;  % 1 for open, 2 for close
    
    % Set correct values for speed, torque, and activation
    CmdGoal.Cmd.Speed = 500;  % Set the speed (0-500 recommended)
    CmdGoal.Cmd.MaxTorquePercentage = 100;  % Set maximum torque to 100%
    CmdGoal.Cmd.HoldTorquePercentage = 100;  % Set hold torque to 100%
    CmdGoal.Cmd.Activate = 1;  % Activate the command

    % Set unique Goal ID for each command
    CmdMsg2.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Stamp = rostime("now") + rosduration(0.05);
    
    % Set unique IDs for opening or closing the gripper
    if action == 1
        CmdMsg2.GoalId.Id = "OPENGRIPPER";
        disp('Opening the gripper...');
    else
        CmdMsg2.GoalId.Id = "CLOSEGRIPPER";
        disp('Closing the gripper...');
    end
    
    % Assign the goal to the message
    CmdMsg2.Goal = CmdGoal;
    
    % Send the message to execute the action
    send(NedCmd2, CmdMsg2);
end
