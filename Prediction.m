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
trainedModelPath = "C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Trained_Model\shape_classification_model_resnet50.mat";
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

% Subscribe to the current joint states
jointStateSub = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");

% Camera topic to display images
cameraSub = rossubscriber("/niryo_robot_vision/compressed_video_stream", "sensor_msgs/CompressedImage");

% Define home and work environment positions
home_position = [0.150, 0, 0.200];
home_orientation = [0, 0.77, 0];

work_position = [0, -0.310, 0.150];
work_orientation = [0, 1.540, 0];

% Move to Home Position
disp('Moving to Home Position...');
move(ned, home_position, home_orientation, jointStateSub);

% Move to Work Environment Position
disp('Moving to Work Environment POV Camera Position...');
move(ned, work_position, work_orientation, jointStateSub);

% Ensure the camera feed is active before capturing
cameraActiveSub = rossubscriber("/niryo_robot_vision/video_stream_is_active");
cameraActiveMsg = receive(cameraActiveSub, 10);
if cameraActiveMsg.Data
    % Capture the image from the camera
    disp('Capturing image...');
    try
        cameraMsg = receive(cameraSub, 10);  % Timeout of 10 seconds
        
        % Extract the compressed image data (in JPEG format)
        imgData = cameraMsg.Data;  % Image data is of type uint8

        % Create a temporary file to store the compressed image
        tempFile = tempname;  % Create a temporary file
        fid = fopen([tempFile, '.jpg'], 'w');  % Open the temp file to write the image
        fwrite(fid, imgData);  % Write the image data to file
        fclose(fid);  % Close the file

        % Read the image back using imread
        img = imread([tempFile, '.jpg']);
        
        % Resize the image to match the network's input size
        imgResized = imresize(img, [224, 224]);  % Resize to 224x224 for the network
        
        % Classify the image using the pre-trained network
        predictedLabel = classify(net, imgResized);

        % Display the image with the predicted label
        imshow(img);
        title(['Predicted Shape: ', char(predictedLabel)]);  % Overlay label on the image
        drawnow;
        
    catch ME
        disp(['Error capturing image: ', ME.message]);
        error('Could not retrieve camera feed. Ensure the camera is active and connected.');
    end
else
    error('Camera feed is not active.');
end

% Move back to Home Position after capturing the image
disp('Moving back to Home Position...');
move(ned, home_position, home_orientation, jointStateSub);

% Shutdown ROS after use
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
